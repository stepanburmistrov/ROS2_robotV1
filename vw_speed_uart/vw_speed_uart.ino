/****************************************************************************
 * Параметры системы
 ***************************************************************************/
#define LF 4    // Цифровой выход (левый мотор): направление
#define LB 5    // Цифровой выход PWM (левый мотор)
#define LE1 2   // Энкодер левого мотора, канал A
#define LE2 A0  // Энкодер левого мотора, канал B

#define RF 7    // Цифровой выход (правый мотор): направление
#define RB 6    // Цифровой выход PWM (правый мотор)
#define RE1 3   // Энкодер правого мотора, канал A
#define RE2 A1  // Энкодер правого мотора, канал B

// Параметры колёс и энкодеров
#define wDiameter    0.069f
#define wResolution  330.0f
#define updateTime   40000UL  // ~20 Гц (50 мс)

// Расстояние между колёсами (мм)
#define TRACK_MM     150.0f

/****************************************************************************
 * Глобальные переменные (позиции, скорость и т.д.)
 ***************************************************************************/
volatile long lpos = 0;  
long lastPos = 0;
uint32_t lastTime = 0;
float vlSpeedFiltered = 0.0f;

volatile long rpos = 0;  
long lastRPos = 0;
uint32_t lastTimeR = 0;
float vrSpeedFiltered = 0.0f;

/****************************************************************************
 * PID-параметры
 ***************************************************************************/
float Kp = 0.9f;
float Ki = 0.5f;
float Kd = 0.0f;

// Левый PID
float integralLeft  = 0.0f;
float prevErrorLeft = 0.0f;

// Правый PID
float integralRight  = 0.0f;
float prevErrorRight = 0.0f;

// Экспоненциальная фильтрация
const float alpha = 0.5f;

// Feedforward
float Kff = 2.2f;

// Для PID
uint32_t lastTimePID_L = 0;
uint32_t lastTimePID_R = 0;

// Выходы PID
float outputLeft  = 0.0f;
float outputRight = 0.0f;

/****************************************************************************
 * Целевые скорости колёс (мм/с)
 ***************************************************************************/
float targetSpeedLeft  = 0.0f;
float targetSpeedRight = 0.0f;

/****************************************************************************
 * Переменные для straight-line по энкодерам
 ***************************************************************************/
bool goingStraight = false;  // флаг: едем ли сейчас по прямой
long leftBase = 0;           // зафиксированный lpos, когда начали ехать прямо
long rightBase = 0;          // зафиксированный rpos, когда начали ехать прямо

/****************************************************************************
 * Для отладочного вывода
 ***************************************************************************/
uint32_t printTimer = 0;

/****************************************************************************
 * Прерывания энкодеров
 ***************************************************************************/
void le_interrupt() {
  if (digitalRead(LE2)) {
    lpos += 1;
  } else {
    lpos -= 1;
  }
}

void re_interrupt() {
  if (digitalRead(RE2)) {
    rpos -= 1;
  } else {
    rpos += 1;
  }
}

/****************************************************************************
 * setup()
 ***************************************************************************/
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("System started. Waiting for commands...");

  pinMode(LE1, INPUT);
  pinMode(LE2, INPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);

  pinMode(RE1, INPUT);
  pinMode(RE2, INPUT);
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LE1), le_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RE1), re_interrupt, RISING);

  digitalWrite(LF, LOW);
  digitalWrite(RF, LOW);

  lastTime      = micros();
  lastTimeR     = micros();
  lastTimePID_L = micros();
  lastTimePID_R = micros();
}

/****************************************************************************
 * Вычисление скорости колёс
 ***************************************************************************/
void computeSpeedLeft() {
  uint32_t now = micros();
  if (now - lastTime >= updateTime) {
    float dt = (float)(now - lastTime) * 1e-6f;

    noInterrupts();
    long currentPos = lpos;
    interrupts();

    long delta = currentPos - lastPos;
    lastPos = currentPos;

    float wlSpeed = ((float)delta / wResolution) * 360.0f / dt;
    float vlSpeed = (wlSpeed / 360.0f) * (PI * wDiameter);
    vlSpeedFiltered = alpha * vlSpeed + (1.0f - alpha) * vlSpeedFiltered;

    lastTime = now;
  }
}

void computeSpeedRight() {
  uint32_t now = micros();
  if (now - lastTimeR >= updateTime) {
    float dt = (float)(now - lastTimeR) * 1e-6f;

    noInterrupts();
    long currentPos = rpos;
    interrupts();

    long delta = currentPos - lastRPos;
    lastRPos = currentPos;

    float wrSpeed = ((float)delta / wResolution) * 360.0f / dt;
    float vrSpeed = (wrSpeed / 360.0f) * (PI * wDiameter);
    vrSpeedFiltered = alpha * vrSpeed + (1.0f - alpha) * vrSpeedFiltered;

    lastTimeR = now;
  }
}

/****************************************************************************
 * setMotorOutput()
 ***************************************************************************/
void setMotorOutput(float output, uint8_t pinDir, uint8_t pinPWM) {
  if (output >= 0) {
    digitalWrite(pinDir, LOW);
    if (output > 255) output = 255;
    analogWrite(pinPWM, (int)output);
  } else {
    digitalWrite(pinDir, HIGH);
    float absVal = -output;
    if (absVal > 255) absVal = 255;
    analogWrite(pinPWM, 255 - (int)absVal);
  }
}

/****************************************************************************
 * Установка (v, w): линейная (мм/с) и угловая (рад/с)
 ***************************************************************************/
void setRobotVelocity(float lin_mm_s, float ang_rad_s) {
  targetSpeedLeft  = lin_mm_s - (ang_rad_s * TRACK_MM * 0.5f);
  targetSpeedRight = lin_mm_s + (ang_rad_s * TRACK_MM * 0.5f);

  // Определяем, едем ли мы прямо (ang = 0)
  if (fabs(ang_rad_s) < 0.0001f) {
    // Считаем, что мы хотим ехать строго прямо.
    goingStraight = true;
    // Зафиксируем текущие значения энкодеров как базу
    leftBase = lpos;
    rightBase = rpos;
  } else {
    goingStraight = false;
  }

  Serial.print(">> setRobotVelocity: lin=");
  Serial.print(lin_mm_s);
  Serial.print(", ang=");
  Serial.print(ang_rad_s);
  Serial.print(" -> vL=");
  Serial.print(targetSpeedLeft);
  Serial.print(", vR=");
  Serial.println(targetSpeedRight);
}

/****************************************************************************
 * loop()
 ***************************************************************************/
void loop() {
  // 1. Команды из Serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // 2. Обновляем скорость колёс
  computeSpeedLeft();
  computeSpeedRight();

  // 3. Считаем PID
  updatePIDForMotors();

  // 4. Периодический вывод
  if (millis() - printTimer > 100) {
    float currentSpeedLeft  = vlSpeedFiltered * 1000.0f;
    float currentSpeedRight = vrSpeedFiltered * 1000.0f;

    Serial.print(F("L Tgt:"));
    Serial.print(targetSpeedLeft, 1);
    Serial.print(F(", Cur:"));
    Serial.print(currentSpeedLeft, 1);
    Serial.print(F(", Out:"));
    Serial.print(outputLeft, 1);

    Serial.print(F(" | R Tgt:"));
    Serial.print(targetSpeedRight, 1);
    Serial.print(F(", Cur:"));
    Serial.print(currentSpeedRight, 1);
    Serial.print(F(", Out:"));
    Serial.println(outputRight, 1);

    printTimer = millis();
  }
}

/****************************************************************************
 * Обновляем PID-регуляторы для каждого колеса
 * + дополнительно "прямолинейная" коррекция по энкодерам
 ***************************************************************************/
void updatePIDForMotors() {
  // 1) Левый мотор
  {
    uint32_t nowPID = micros();
    float dt = (float)(nowPID - lastTimePID_L) * 1e-6f;
    lastTimePID_L = nowPID;

    float currentSpeedLeft = vlSpeedFiltered * 1000.0f; // мм/с
    float errorLeft = targetSpeedLeft - currentSpeedLeft;

    float feedforwardLeft = targetSpeedLeft / Kff;

    // Интеграл
    if (outputLeft < 253 && outputLeft > -253) {
      integralLeft += errorLeft * dt;
    }
    // Ограничение
    if (integralLeft > 100.0f)  integralLeft = 100.0f;
    if (integralLeft < -100.0f) integralLeft = -100.0f;

    float derivativeLeft = (errorLeft - prevErrorLeft) / dt;
    prevErrorLeft = errorLeft;

    float pidOutputLeft = Kp * errorLeft + Ki * integralLeft + Kd * derivativeLeft;
    outputLeft = feedforwardLeft + pidOutputLeft;

    // Если целевая скорость ~0, обнулим
    if (fabs(targetSpeedLeft) < 1.0f) {
      integralLeft = 0.0f;
      outputLeft   = 0.0f;
    }
  }

  // 2) Правый мотор
  {
    uint32_t nowPID = micros();
    float dt = (float)(nowPID - lastTimePID_R) * 1e-6f;
    lastTimePID_R = nowPID;

    float currentSpeedRight = vrSpeedFiltered * 1000.0f;
    float errorRight = targetSpeedRight - currentSpeedRight;

    float feedforwardRight = targetSpeedRight / Kff;

    if (outputRight < 253 && outputRight > -253) {
      integralRight += errorRight * dt;
    }
    if (integralRight > 100.0f)  integralRight = 100.0f;
    if (integralRight < -100.0f) integralRight = -100.0f;

    float derivativeRight = (errorRight - prevErrorRight) / dt;
    prevErrorRight = errorRight;

    float pidOutputRight = Kp * errorRight + Ki * integralRight + Kd * derivativeRight;
    outputRight = feedforwardRight + pidOutputRight;

    if (fabs(targetSpeedRight) < 1.0f) {
      integralRight = 0.0f;
      outputRight   = 0.0f;
    }
  }

  // --- STRAIGHT-LINE CORRECTION BY ENCODERS ---
  // Если мы "едем прямо"
  if (goingStraight and fabs(targetSpeedRight) > 1.0f and fabs(targetSpeedLeft) > 1.0f) {
    // Текущее накопленное смещение левого и правого колеса 
    long leftTicks  = lpos  - leftBase;
    long rightTicks = rpos - rightBase;
    // diff > 0 => левое колесо прошло больше тиков
    long diff = (leftTicks - rightTicks);

    // Выберем небольшой коэффициент
    float KencStraight = 10.0f;  // подберите опытным путём

    // Коррекция (как пропорциональный регулятор)
    float corr = KencStraight * (float)diff;

    // "Ополовиниваем" коррекцию, чтобы не было слишком агрессивно
    outputLeft  -= corr * 0.5f;
    outputRight += corr * 0.5f;
  }
  // --- END STRAIGHT-LINE CORRECTION ---

  // Подаём итог на моторы
  setMotorOutput(outputLeft, LF, LB);
  setMotorOutput(outputRight, RF, RB);
}

/****************************************************************************
 * ОБРАБОТКА КОМАНД из Serial
 ***************************************************************************/
void processCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
    return;
  } else if (command.startsWith("SET_VELOCITY")) {
    float lin = 0.0f, ang = 0.0f;
    if (parseSetVelocityCommand(command, &lin, &ang)) {
      setRobotVelocity(lin, ang);
      Serial.print("OK: Set velocity to ");
      Serial.print(lin);
      Serial.print(" mm/s, ");
      Serial.print(ang);
      Serial.println(" rad/s");
    }
    return;
  } else {
    Serial.println("ERROR: Unknown command");
  }
}

/****************************************************************************
 * Парсер команды "SET_VELOCITY lin ang"
 ***************************************************************************/
bool parseSetVelocityCommand(const String& command, float* linear, float* angular) {
  int index = command.indexOf(' ');
  if (index == -1) {
    Serial.println("ERROR: Invalid SET_VELOCITY command");
    return false;
  }
  String params = command.substring(index + 1);
  params.trim();

  int idxSpace = params.indexOf(' ');
  if (idxSpace == -1) {
    Serial.println("ERROR: Missing parameters for SET_VELOCITY");
    return false;
  }
  String linStr = params.substring(0, idxSpace);
  String angStr = params.substring(idxSpace + 1);
  linStr.trim();
  angStr.trim();

  float linVal = linStr.toFloat();
  float angVal = angStr.toFloat();
  *linear = linVal;
  *angular = angVal;

  return true;
}
