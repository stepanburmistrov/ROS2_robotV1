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

const float WHEEL_DIAMETER = 69.0;         // мм
const float WHEEL_BASE = 185.0;            // мм (расстояние между центрами колёс)
const float ENCODER_RESOLUTION = 330.0;    // тиков на оборот
const float TICKS_TO_MM = (PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
const float ALPHA = 0.5; // Коэффициент экспоненциальной фильтрации скорости

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
    resetPID();
    lastTargetLeft = targetLeftWheelSpeed;
  }
  if (targetRightWheelSpeed != lastTargetRight) {
    resetPID();
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
  float errorLeft  = targetLeftWheelSpeed  - vlSpeedFiltered;
  float errorRight = targetRightWheelSpeed - vrSpeedFiltered;

  errorLeftIntegral  += errorLeft  * dt;
  errorRightIntegral += errorRight * dt;
  if (errorLeftIntegral > integralLimit)  errorLeftIntegral  = integralLimit;
  if (errorLeftIntegral < -integralLimit) errorLeftIntegral  = -integralLimit;
  if (errorRightIntegral > integralLimit)  errorRightIntegral = integralLimit;
  if (errorRightIntegral < -integralLimit) errorRightIntegral = -integralLimit;

  float derivativeLeft  = (errorLeft  - prevErrorLeft)  / dt;
  float derivativeRight = (errorRight - prevErrorRight) / dt;
  prevErrorLeft  = errorLeft;
  prevErrorRight = errorRight;

  float pidLeft  = pidKp * errorLeft  + pidKi * errorLeftIntegral  + pidKd * derivativeLeft;
  float pidRight = pidKp * errorRight + pidKi * errorRightIntegral + pidKd * derivativeRight;

  float outputLeft  = pidLeft  + pidKff * targetLeftWheelSpeed;
  float outputRight = pidRight + pidKff * targetRightWheelSpeed;

  // --- Компенсация прямолинейного движения ---
  if (goingStraight && fabs(targetLeftWheelSpeed) > 1.0 && fabs(targetRightWheelSpeed) > 1.0) {
    // Вычисляем накопленное смещение от базового значения, зафиксированного при старте прямолинейного движения
    long leftTicks  = left_encoder_value - leftBase;
    long rightTicks = right_encoder_value - rightBase;
    long diff = leftTicks - rightTicks;  // если diff > 0, левое колесо продвинулось дальше
    float KencStraight = 3.0f;  // Подберите опытным путём
    float corr = KencStraight * (float)diff;
    // Применяем корректировку: по 50% для каждого колеса
    outputLeft  -= corr * 0.5f;
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
  leftA  = constrain(leftA, 0, 255);
  leftB  = constrain(leftB, 0, 255);
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

bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if (index3 == -1) return false;
  int index4 = command.indexOf(' ', index3 + 1);
  if (index4 == -1) return false;
  *leftA_PWM = command.substring(index1 + 1, index2).toInt();
  *leftB_PWM = command.substring(index2 + 1, index3).toInt();
  *rightA_PWM = command.substring(index3 + 1, index4).toInt();
  *rightB_PWM = command.substring(index4 + 1).toInt();
  return true;
}

bool parseSetSpeed(const String& command, int* speedLeft, int* speedRight) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  *speedLeft = command.substring(index1 + 1, index2).toInt();
  *speedRight = command.substring(index2 + 1).toInt();
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

/****************************************************************************
 * Обработка команд из UART
 ***************************************************************************/
void processCommand(String command) {
  command.trim();
  if (command.startsWith("SET_PWM")) {
    int leftA_PWM = 0, leftB_PWM = 0, rightA_PWM = 0, rightB_PWM = 0;
    if (parseSetPWM(command, &leftA_PWM, &leftB_PWM, &rightA_PWM, &rightB_PWM)) {
      setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
      Serial.println("OK: Set PWM");
    }
  } else if (command.startsWith("SET_POSE")) {
    float x, y, th;
    // Парсинг команды SET_POSE (аналог предыдущих примеров)
    Serial.println("OK: Pose set");
  } else if (command.startsWith("SET_WHEELS_SPEED")) {
    int leftSpeed = 0, rightSpeed = 0;
    if (parseSetSpeed(command, &leftSpeed, &rightSpeed)) {
      targetLeftWheelSpeed = leftSpeed;
      targetRightWheelSpeed = rightSpeed;
      Serial.println("OK: Wheel speed set");
    }
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
  } else {
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
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  static uint32_t PIDTimer = 0;
  if (millis() - PIDTimer > 50) {
    computeSpeed();
    updateOdometry();
    computeWheelsPID();
    PIDTimer = millis();
  }

  static uint32_t printTimer = 0;
  if (millis() - printTimer > 200) {
    Serial.printf("POS X=%.2f Y=%.2f Th=%.2f ", xPos, yPos, theta);
    Serial.printf("ENC L=%ld R=%ld ", left_encoder_value, right_encoder_value);
    Serial.printf("SPD L=%.2f mm/s R=%.2f mm/s ", vlSpeedFiltered, vrSpeedFiltered);
    Serial.printf("Target L=%.2f mm/s R=%.2f mm/s\n", targetLeftWheelSpeed, targetRightWheelSpeed);
    printTimer = millis();
  }
}