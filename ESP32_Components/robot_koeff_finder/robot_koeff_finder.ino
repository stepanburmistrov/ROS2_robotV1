// Определение пинов, переменных энкодеров, положения робота и т.д.
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
float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;
float vlSpeedFiltered = 0.0, vrSpeedFiltered = 0.0;
uint32_t lastTimeL = 0, lastTimeR = 0;

float targetLeftWheelSpeed = 0, targetRightWheelSpeed = 0;

float xPos = 0.0, yPos = 0.0, theta = 0.0;

const float WHEEL_DIAMETER = 69.0;
const float WHEEL_BASE = 185.0;
const float ENCODER_RESOLUTION = 330.0;
const float TICKS_TO_MM = (PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
const float ALPHA = 0.5; // Коэффициент фильтрации скорости

// Глобальные коэффициенты PID – их можно обновлять через UART
float pidKp = 1.1;
float pidKi = 1.3;
float pidKd = 0.01;
float pidKff = 0.25;

void left_interrupt() {
  digitalRead(LEFT_ENCODER_B) ? left_encoder_value++ : left_encoder_value--;
}
void right_interrupt() {
  digitalRead(RIGHT_ENCODER_B) ? right_encoder_value++ : right_encoder_value--;
}

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

void computeSpeed() {
  uint32_t now = micros();

  if (lastTimeL == 0 || lastTimeR == 0) { 
    lastTimeL = now;
    lastTimeR = now;
    last_speed_left_encoder = left_encoder_value;
    last_speed_right_encoder = right_encoder_value;
    return;
  }

  float dtL = (now - lastTimeL) * 0.000001;
  float dtR = (now - lastTimeR) * 0.000001;

  if (dtL > 0 && left_encoder_value != last_speed_left_encoder) {
    long deltaLeft = left_encoder_value - last_speed_left_encoder;
    float wlSpeed = ((float)deltaLeft / ENCODER_RESOLUTION) * 360.0f / dtL;
    float vlSpeed = (wlSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
    vlSpeedFiltered = ALPHA * vlSpeed + (1.0f - ALPHA) * vlSpeedFiltered;
    last_speed_left_encoder = left_encoder_value;
    lastTimeL = now;
  } else if (dtL > 0.005 && left_encoder_value == last_speed_left_encoder){
    vlSpeedFiltered = 0;
  }

  if (dtR > 0 && right_encoder_value != last_speed_right_encoder) {
    long deltaRight = right_encoder_value - last_speed_right_encoder;
    float wrSpeed = ((float)deltaRight / ENCODER_RESOLUTION) * 360.0f / dtR;
    float vrSpeed = (wrSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
    vrSpeedFiltered = ALPHA * vrSpeed + (1.0f - ALPHA) * vrSpeedFiltered;
    last_speed_right_encoder = right_encoder_value;
    lastTimeR = now;
  } else if (dtR > 0.005 && right_encoder_value == last_speed_right_encoder){
    vrSpeedFiltered = 0;
  }
}

void computeWheelsPID(){
  // Ограничение интегральной составляющей (anti-windup)
  const float integralLimit = 100.0;

  // Статические переменные для состояния PID для каждого колеса
  static float errorLeftIntegral = 0;
  static float errorRightIntegral = 0;
  static float prevErrorLeft = 0;
  static float prevErrorRight = 0;
  static uint32_t lastPIDTime = millis();
  // Для сброса интегральной составляющей при смене целевой скорости
  static float lastTargetLeft = 0.0;
  static float lastTargetRight = 0.0;

  // Сброс интеграла, если целевая скорость изменилась
  if(targetLeftWheelSpeed != lastTargetLeft){
    errorLeftIntegral = 0;
    prevErrorLeft = 0;
    lastTargetLeft = targetLeftWheelSpeed;
  }
  if(targetRightWheelSpeed != lastTargetRight){
    errorRightIntegral = 0;
    prevErrorRight = 0;
    lastTargetRight = targetRightWheelSpeed;
  }

  // Вычисляем интервал dt (в секундах)
  uint32_t now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  if(dt < 0.001f) dt = 0.001f;
  lastPIDTime = now;

  // Расчет ошибок
  float errorLeft  = targetLeftWheelSpeed  - vlSpeedFiltered;
  float errorRight = targetRightWheelSpeed - vrSpeedFiltered;

  // Интегральная составляющая
  errorLeftIntegral  += errorLeft  * dt;
  errorRightIntegral += errorRight * dt;
  if(errorLeftIntegral > integralLimit)  errorLeftIntegral  = integralLimit;
  if(errorLeftIntegral < -integralLimit) errorLeftIntegral  = -integralLimit;
  if(errorRightIntegral > integralLimit)  errorRightIntegral = integralLimit;
  if(errorRightIntegral < -integralLimit) errorRightIntegral = -integralLimit;

  // Производная ошибки
  float derivativeLeft  = (errorLeft  - prevErrorLeft)  / dt;
  float derivativeRight = (errorRight - prevErrorRight) / dt;
  prevErrorLeft  = errorLeft;
  prevErrorRight = errorRight;

  // Вычисление PID-выхода с использованием глобальных коэффициентов
  float pidLeft  = pidKp * errorLeft  + pidKi * errorLeftIntegral  + pidKd * derivativeLeft;
  float pidRight = pidKp * errorRight + pidKi * errorRightIntegral + pidKd * derivativeRight;

  // Добавляем feedforward
  float outputLeft  = pidLeft  + pidKff * targetLeftWheelSpeed;
  float outputRight = pidRight + pidKff * targetRightWheelSpeed;

  // Преобразование в значения ШИМ
  int leftA_PWM = 0, leftB_PWM = 0;
  int rightA_PWM = 0, rightB_PWM = 0;

  if(outputLeft >= 0){
    leftA_PWM = 0;
    leftB_PWM = constrain((int)outputLeft, 0, 255);
  } else {
    leftA_PWM = constrain((int)(-outputLeft), 0, 255);
    leftB_PWM = 0;
  }
  if(outputRight >= 0){
    rightA_PWM = 0;
    rightB_PWM = constrain((int)outputRight, 0, 255);
  } else {
    rightA_PWM = constrain((int)(-outputRight), 0, 255);
    rightB_PWM = 0;
  }

  setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
}

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

// Функция для парсинга команды установки коэффициентов: "SET_COEFF Kp Ki Kd Kff"
bool parseSetCoeff(const String& command, float* Kp, float* Ki, float* Kd, float* Kff) {
  int index1 = command.indexOf(' ');
  if(index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if(index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if(index3 == -1) return false;
  int index4 = command.indexOf(' ', index3 + 1);
  if(index4 == -1) return false;

  *Kp = command.substring(index1 + 1, index2).toFloat();
  *Ki = command.substring(index2 + 1, index3).toFloat();
  *Kd = command.substring(index3 + 1, index4).toFloat();
  *Kff = command.substring(index4 + 1).toFloat();
  return true;
}

// Другие функции парсинга команд (SET_PWM, SET_POSE, SET_WHEELS_SPEED) аналогичны предыдущим примерам
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

void processCommand(String command) {
  command.trim();
  if (command.startsWith("SET_PWM")) {
    int leftA_PWM = 0, leftB_PWM = 0, rightA_PWM =0, rightB_PWM = 0;
    if (parseSetPWM(command, &leftA_PWM, &leftB_PWM, &rightA_PWM, &rightB_PWM)) {
      setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
      Serial.println("OK: Set PWM");
    }
  } else if (command.startsWith("SET_POSE")) {
    float x, y, th;
    // Парсинг команды SET_POSE аналогичен предыдущему примеру
    // …
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
  } else {
    Serial.println("ERROR: Unknown command");
  }
}

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