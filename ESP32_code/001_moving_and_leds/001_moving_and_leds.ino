#include <Adafruit_NeoPixel.h>
#include <SCServo.h>
// Класс для управления STS-сервами (FEETECH STS3215 и т.д.)
SMS_STS st;

// Пины UART (приведён пример для ESP32 с UART1, GPIO 18/19).
#define S_RXD 18
#define S_TXD 19

// Скорость UART для серво
#define SERVO_BAUD 1000000

// ------------------------- НАСТРОЙКИ СВЕТОДИОДНОЙ ЛЕНТЫ -------------------------
#define PIN_WS2812B 14     // Пин, к которому подключена лента
#define NUM_PIXELS 50      // Кол-во светодиодов в ленте
#define LED_BRIGHTNESS 50  // Базовая «глобальная» яркость (0-255), если нужно

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

uint8_t bootMode = 1;
// ---------------------------------------------------------------------------------

/****************************************************************************
 * Определение пинов, переменных энкодеров, положения робота и т.д.
 ***************************************************************************/
#define LEFT_MOTOR_A 27
#define LEFT_MOTOR_B 26
#define RIGHT_MOTOR_A 25
#define RIGHT_MOTOR_B 33

#define LEFT_ENCODER_A 39
#define LEFT_ENCODER_B 34
#define RIGHT_ENCODER_A 32
#define RIGHT_ENCODER_B 35

volatile long left_encoder_value = 0;
volatile long right_encoder_value = 0;
long last_left_encoder = 0;
long last_right_encoder = 0;

long last_speed_left_encoder = 0, last_speed_right_encoder = 0;
float vlSpeedFiltered = 0.0, vrSpeedFiltered = 0.0;
uint32_t lastTimeL = 0, lastTimeR = 0;

float targetLeftWheelSpeed = 0, targetRightWheelSpeed = 0;

float xPos = 0.0, yPos = 0.0, theta = 0.0;

const float WHEEL_DIAMETER = 69.0;       // мм
const float WHEEL_BASE = 185.0;          // мм (расстояние между центрами колёс)
const float ENCODER_RESOLUTION = 330.0;  // тиков на оборот
const float TICKS_TO_MM = (PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
const float ALPHA = 0.5;  // Коэффициент экспоненциальной фильтрации скорости

// Глобальные коэффициенты PID – их можно обновлять через UART
float pidKp = 1.1;
float pidKi = 1.3;
float pidKd = 0.01;
float pidKff = 0.25;

// Глобальные PID-состояния
float errorLeftIntegral = 0;
float errorRightIntegral = 0;
float prevErrorLeft = 0;
float prevErrorRight = 0;
uint32_t lastPIDTime = 0;

// Функция сброса PID-переменных (интегральная и дифференциальная составляющие, а также фильтрованные скорости)
void resetPID() {
  errorLeftIntegral = 0;
  errorRightIntegral = 0;
  prevErrorLeft = 0;
  prevErrorRight = 0;
  vlSpeedFiltered = 0;
  vrSpeedFiltered = 0;
  lastPIDTime = millis();
}

/****************************************************************************
 * Переменные компенсации прямолинейного движения
 ***************************************************************************/
bool goingStraight = false;  // Флаг, указывающий, что робот едет прямо (угловая скорость ≈ 0)
long leftBase = 0;           // Значение left_encoder_value в момент начала прямолинейного движения
long rightBase = 0;          // Значение right_encoder_value в момент начала прямолинейного движения

/****************************************************************************
 * Прерывания энкодеров
 ***************************************************************************/
void left_interrupt() {
  digitalRead(LEFT_ENCODER_B) ? left_encoder_value++ : left_encoder_value--;
}
void right_interrupt() {
  digitalRead(RIGHT_ENCODER_B) ? right_encoder_value++ : right_encoder_value--;
}

/****************************************************************************
 * Одометрия
 ***************************************************************************/
void updateOdometry() {
  long deltaLeft = left_encoder_value - last_left_encoder;
  long deltaRight = right_encoder_value - last_right_encoder;
  last_left_encoder = left_encoder_value;
  last_right_encoder = right_encoder_value;

  float distLeft = deltaLeft * TICKS_TO_MM;
  float distRight = deltaRight * TICKS_TO_MM;
  float deltaS = (distLeft + distRight) / 2.0;
  float deltaTheta = (distRight - distLeft) / WHEEL_BASE;

  theta += deltaTheta;
  xPos += deltaS * cos(theta);
  yPos += deltaS * sin(theta);
}

/****************************************************************************
 * Вычисление скорости колес
 ***************************************************************************/
void computeSpeed() {
  uint32_t now = micros();
  if (lastTimeL == 0 || lastTimeR == 0) {
    lastTimeL = now;
    lastTimeR = now;
    last_speed_left_encoder = left_encoder_value;
    last_speed_right_encoder = right_encoder_value;
    return;
  }
  float dtL = (now - lastTimeL) * 1e-6f;
  float dtR = (now - lastTimeR) * 1e-6f;

  if (dtL > 0 && left_encoder_value != last_speed_left_encoder) {
    long deltaLeft = left_encoder_value - last_speed_left_encoder;
    float wlSpeed = ((float)deltaLeft / ENCODER_RESOLUTION) * 360.0f / dtL;
    float vlSpeed = (wlSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
    vlSpeedFiltered = ALPHA * vlSpeed + (1.0f - ALPHA) * vlSpeedFiltered;
    last_speed_left_encoder = left_encoder_value;
    lastTimeL = now;
  } else if (dtL > 0.005 && left_encoder_value == last_speed_left_encoder) {
    vlSpeedFiltered = 0;
  }
  if (dtR > 0 && right_encoder_value != last_speed_right_encoder) {
    long deltaRight = right_encoder_value - last_speed_right_encoder;
    float wrSpeed = ((float)deltaRight / ENCODER_RESOLUTION) * 360.0f / dtR;
    float vrSpeed = (wrSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
    vrSpeedFiltered = ALPHA * vrSpeed + (1.0f - ALPHA) * vrSpeedFiltered;
    last_speed_right_encoder = right_encoder_value;
    lastTimeR = now;
  } else if (dtR > 0.005 && right_encoder_value == last_speed_right_encoder) {
    vrSpeedFiltered = 0;
  }
}

/****************************************************************************
 * PID-регулятор для колес с компенсацией прямолинейного движения
 ***************************************************************************/
void computeWheelsPID() {
  const float integralLimit = 30.0;
  static float lastTargetLeft = 0.0;
  static float lastTargetRight = 0.0;

  // Если целевая скорость изменилась или установлена остановка – сброс PID
  if (targetLeftWheelSpeed != lastTargetLeft) {
    lastTargetLeft = targetLeftWheelSpeed;
  }
  if (targetRightWheelSpeed != lastTargetRight) {
    lastTargetRight = targetRightWheelSpeed;
  }
  if (targetLeftWheelSpeed == 0 && targetRightWheelSpeed == 0) {
    resetPID();
  }

  uint32_t now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastPIDTime = now;

  // Вычисляем ошибки между целевыми и измеренными скоростями
  float errorLeft = targetLeftWheelSpeed - vlSpeedFiltered;
  float errorRight = targetRightWheelSpeed - vrSpeedFiltered;

  errorLeftIntegral += errorLeft * dt;
  errorRightIntegral += errorRight * dt;
  if (errorLeftIntegral > integralLimit) errorLeftIntegral = integralLimit;
  if (errorLeftIntegral < -integralLimit) errorLeftIntegral = -integralLimit;
  if (errorRightIntegral > integralLimit) errorRightIntegral = integralLimit;
  if (errorRightIntegral < -integralLimit) errorRightIntegral = -integralLimit;

  float derivativeLeft = (errorLeft - prevErrorLeft) / dt;
  float derivativeRight = (errorRight - prevErrorRight) / dt;
  prevErrorLeft = errorLeft;
  prevErrorRight = errorRight;

  float pidLeft = pidKp * errorLeft + pidKi * errorLeftIntegral + pidKd * derivativeLeft;
  float pidRight = pidKp * errorRight + pidKi * errorRightIntegral + pidKd * derivativeRight;

  float outputLeft = pidLeft + pidKff * targetLeftWheelSpeed;
  float outputRight = pidRight + pidKff * targetRightWheelSpeed;

  // --- Компенсация прямолинейного движения ---
  if (goingStraight && fabs(targetLeftWheelSpeed) > 1.0 && fabs(targetRightWheelSpeed) > 1.0) {
    // Вычисляем накопленное смещение от базового значения, зафиксированного при старте прямолинейного движения
    long leftTicks = left_encoder_value - leftBase;
    long rightTicks = right_encoder_value - rightBase;
    long diff = leftTicks - rightTicks;  // если diff > 0, левое колесо продвинулось дальше
    float KencStraight = 3.0f;           // Подберите опытным путём
    float corr = KencStraight * (float)diff;
    // Применяем корректировку: по 50% для каждого колеса
    outputLeft -= corr * 0.5f;
    outputRight += corr * 0.5f;
  }
  // --- Конец компенсации ---

  int leftA_PWM = 0, leftB_PWM = 0;
  int rightA_PWM = 0, rightB_PWM = 0;

  if (outputLeft >= 0) {
    leftA_PWM = 0;
    leftB_PWM = constrain((int)outputLeft, 0, 255);
  } else {
    leftA_PWM = constrain((int)(-outputLeft), 0, 255);
    leftB_PWM = 0;
  }
  if (outputRight >= 0) {
    rightA_PWM = 0;
    rightB_PWM = constrain((int)outputRight, 0, 255);
  } else {
    rightA_PWM = constrain((int)(-outputRight), 0, 255);
    rightB_PWM = 0;
  }

  setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
}

/****************************************************************************
 * Функция управления моторами (вывод ШИМ)
 ***************************************************************************/
void setMotorsPWM(int leftA, int leftB, int rightA, int rightB) {
  leftA = constrain(leftA, 0, 255);
  leftB = constrain(leftB, 0, 255);
  rightA = constrain(rightA, 0, 255);
  rightB = constrain(rightB, 0, 255);
  analogWrite(LEFT_MOTOR_A, leftA);
  analogWrite(LEFT_MOTOR_B, leftB);
  analogWrite(RIGHT_MOTOR_A, rightA);
  analogWrite(RIGHT_MOTOR_B, rightB);
}

/****************************************************************************
 * Функция для установки скорости робота (линейной и угловой)
 * linearVelocity в мм/с, angularVelocity в рад/с
 ***************************************************************************/
void setRobotVelocity(float linearVelocity, float angularVelocity) {
  // Если оба параметра (линейная и угловая скорости) близки к нулю – остановка робота
  if (fabs(linearVelocity) < 10 && fabs(angularVelocity) < 0.1) {
    targetLeftWheelSpeed = 0;
    targetRightWheelSpeed = 0;
    vlSpeedFiltered = 0;
    vrSpeedFiltered = 0;
    resetPID();
    goingStraight = false;
    return;
  }

  // Если угловая скорость почти нулевая, считаем, что едем прямо
  if (fabs(angularVelocity) < 0.0001f) {
    goingStraight = true;
    leftBase = left_encoder_value;    // фиксируем текущее значение для левого энкодера
    rightBase = right_encoder_value;  // фиксируем текущее значение для правого энкодера
  } else {
    goingStraight = false;
  }

  // Вычисляем целевые скорости для левого и правого колёс по обратной кинематике
  targetLeftWheelSpeed = linearVelocity - (angularVelocity * WHEEL_BASE / 2.0);
  targetRightWheelSpeed = linearVelocity + (angularVelocity * WHEEL_BASE / 2.0);
}

/****************************************************************************
 * Парсинг команд UART
 ***************************************************************************/
bool parseSetCoeff(const String& command, float* Kp, float* Ki, float* Kd, float* Kff) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if (index3 == -1) return false;
  int index4 = command.indexOf(' ', index3 + 1);
  if (index4 == -1) return false;
  *Kp = command.substring(index1 + 1, index2).toFloat();
  *Ki = command.substring(index2 + 1, index3).toFloat();
  *Kd = command.substring(index3 + 1, index4).toFloat();
  *Kff = command.substring(index4 + 1).toFloat();
  return true;
}

bool parseSetPose(const String& command, float* x, float* y, float* th) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if (index3 == -1) return false;

  *x = command.substring(index1 + 1, index2).toFloat();
  *y = command.substring(index2 + 1, index3).toFloat();
  *th = command.substring(index3 + 1).toFloat();
  return true;
}

bool parseSetRobotVelocity(const String& command, float* linearVelocity, float* angularVelocity) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  *linearVelocity = command.substring(index1 + 1, index2).toFloat();
  *angularVelocity = command.substring(index2 + 1).toFloat();
  return true;
}

// ----------------------------------------------------------------------------
// Парсер команды SET_LED_DATA <count> <hexData>
// где <hexData> = count * 4 символов (на каждый пиксель 2 байта = 4 hex-символа).
// Пример: SET_LED_DATA 2 FFFF0F0F
// ----------------------------------------------------------------------------
void parseSetLedDataCommand(String command) {
  // 1) Находим первое пробел
  int index = command.indexOf(' ');
  if (index == -1) {
    Serial.println("ERROR: Invalid SET_LED_DATA command");
    return;
  }

  // Извлекаем подстроку после "SET_LED_DATA "
  String params = command.substring(index + 1);
  params.trim();

  // 2) Находим следующее пробел — отделить count от hexData
  int idxCount = params.indexOf(' ');
  if (idxCount == -1) {
    Serial.println("ERROR: No pixel count in SET_LED_DATA");
    return;
  }

  // Читаем count
  String countStr = params.substring(0, idxCount);
  int pixelCount = countStr.toInt();
  if (pixelCount < 1 || pixelCount > NUM_PIXELS) {
    Serial.println("ERROR: pixelCount out of range");
    return;
  }

  // Оставшаяся часть — hexData
  String hexData = params.substring(idxCount + 1);
  hexData.trim();

  // Проверим длину hexData
  // На каждый пиксель нужно 4 hex-символа (2 байта),
  // значит всего нужно (pixelCount * 4) символов.
  int requiredLen = pixelCount * 4;
  if (hexData.length() < requiredLen) {
    Serial.println("ERROR: hexData too short");
    return;
  }

  // 3) Парсим hexData по 4 символа на каждый пиксель
  for (int i = 0; i < pixelCount; i++) {
    // Читаем 4 hex-символа
    String chunk = hexData.substring(i * 4, i * 4 + 4);

    // Преобразуем в 16-битное число
    uint16_t colorVal = (uint16_t)strtol(chunk.c_str(), NULL, 16);

    // Распаковываем (R4, G4, B4, Br)
    uint8_t R4 = (colorVal >> 12) & 0x0F;  // старшие 4 бита
    uint8_t G4 = (colorVal >> 8) & 0x0F;
    uint8_t B4 = (colorVal >> 4) & 0x0F;
    uint8_t Br = colorVal & 0x0F;

    // Преобразуем в 8-битные
    // Сначала 0..15 конвертируем в 0..255, потом масштабируем на Brightness (Br)
    // Т.е. (R4 * 255/15) -> R8base, затем R8 = R8base * (Br/15)
    uint8_t R8base = (R4 * 255) / 15;
    uint8_t G8base = (G4 * 255) / 15;
    uint8_t B8base = (B4 * 255) / 15;

    uint8_t R8 = (uint8_t)((R8base * Br) / 15);
    uint8_t G8 = (uint8_t)((G8base * Br) / 15);
    uint8_t B8 = (uint8_t)((B8base * Br) / 15);

    // Устанавливаем цвет пикселя в ленте
    ws2812b.setPixelColor(i, ws2812b.Color(R8, G8, B8));
  }

  // Если нужно очистить остаток ленты, если передано < NUM_PIXELS
  for (int i = pixelCount; i < NUM_PIXELS; i++) {
    ws2812b.setPixelColor(i, 0);  // выключим
  }

  // 4) Обновляем ленту одним вызовом для скорости
  ws2812b.show();

  // Сообщаем об успехе
  Serial.println("OK");
}


void waitingAnimation() {
  static uint32_t ledTimer = millis();
  static uint8_t hsv = 0;
  static uint8_t currentPixel = 0;
  if (millis() - ledTimer > 50) {
    ws2812b.setPixelColor(currentPixel, ws2812b.ColorHSV(hsv * 255, 255, 50));
    ws2812b.show();
    hsv += 2;
    currentPixel++;
    if (currentPixel >= NUM_PIXELS) currentPixel = 0;
    ledTimer = millis();
  }
}




/****************************************************************************
 * Обработка команд из UART
 ***************************************************************************/
void processCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("ROBOT_001");
    for (int i = 0; i < NUM_PIXELS; i++) {
      ws2812b.setPixelColor(i, 0);  // выключим ленту
    }
    ws2812b.show();
    bootMode = 0;
  } else if (command.startsWith("SET_POSE")) {
    float x, y, th;
    if (parseSetPose(command, &x, &y, &th)) {
      xPos = x;
      yPos = y;
      theta = th;
      Serial.println("OK: Pose set");
    }
    Serial.println("OK: Pose set");
  } else if (command.startsWith("SET_COEFF")) {
    float newKp, newKi, newKd, newKff;
    if (parseSetCoeff(command, &newKp, &newKi, &newKd, &newKff)) {
      pidKp = newKp;
      pidKi = newKi;
      pidKd = newKd;
      pidKff = newKff;
      Serial.println("OK: Coefficients updated");
    } else {
      Serial.println("ERROR: Invalid coefficients");
    }
  } else if (command.startsWith("SET_ROBOT_VELOCITY")) {
    float linVel = 0.0, angVel = 0.0;
    if (parseSetRobotVelocity(command, &linVel, &angVel)) {
      setRobotVelocity(linVel, angVel);
      Serial.println("OK: Robot velocity set");
    } else {
      Serial.println("ERROR: Invalid robot velocity command");
    }
  } else if (command.startsWith("SET_LED_DATA")) {
    parseSetLedDataCommand(command);
  } 

  else {
    Serial.println("ERROR: Unknown command");
  }
}

/****************************************************************************
 * setup() и loop()
 ***************************************************************************/
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("System started");

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;


  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), left_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), right_interrupt, RISING);

  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);

  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // Инициализируем NeoPixel
  ws2812b.begin();
  ws2812b.setBrightness(LED_BRIGHTNESS);
  ws2812b.clear();
  ws2812b.show();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  updateOdometry();

  static uint32_t PIDTimer = 0;
  if (millis() - PIDTimer > 50) {
    computeSpeed();
    computeWheelsPID();
    PIDTimer = millis();
  }

  static uint32_t printTimer = 0;
  if (millis() - printTimer > 200 and !bootMode) {
    Serial.printf("POS X=%.2f Y=%.2f Th=%.2f ", xPos, yPos, theta);
    Serial.printf("ENC L=%ld R=%ld ", left_encoder_value, right_encoder_value);
    Serial.printf("SPD L=%.2f mm/s R=%.2f mm/s ", vlSpeedFiltered, vrSpeedFiltered);
    Serial.printf("Target L=%.2f mm/s R=%.2f mm/s\n", targetLeftWheelSpeed, targetRightWheelSpeed);
    printTimer = millis();
  }

  if (bootMode == 1) { waitingAnimation(); }
}