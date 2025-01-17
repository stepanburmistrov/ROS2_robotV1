// ------------------------- Настройки -------------------------
#define LEFT_ENCODER_PIN 3   // Пин энкодера левого колеса (прерывание)
#define RIGHT_ENCODER_PIN 2  // Пин энкодера правого колеса (прерывание)

// Пины управления левым мотором
#define LEFT_MOTOR_DIR_PIN 7
#define LEFT_MOTOR_PWM_PIN 6

// Пины управления правым мотором
#define RIGHT_MOTOR_DIR_PIN 4
#define RIGHT_MOTOR_PWM_PIN 5


// Базовая скорость и коэффициент
int basePWM = 200;
float kP = 30;

// Глобальные переменные для подсчёта тиков
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

long integral = 0;


void setup() {
  Serial.begin(9600);

  // Настраиваем пины энкодеров как входы
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  // Настраиваем пины моторов как выходы
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  // Включаем прерывания по CHANGE на энкодерах
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), updateEncoderRight, CHANGE);

  // Настраиваем пины кнопок как вход с подтягивающим резистором
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
}

// Функция прерывания для левого энкодера
void updateEncoderLeft() {
  encoderCountLeft++;
}

// Функция прерывания для правого энкодера
void updateEncoderRight() {
  encoderCountRight++;
}

void loop() {
  // Расчёт ошибки: хотим, чтобы левое и правое колёса вращались с одинаковой скоростью

  long error = encoderCountLeft - encoderCountRight;

  // Пропорциональный коэффициент
  int correction = kP * error;
  // Рассчитываем ШИМ для левого и правого мотора с учётом поправки
  int pwmLeft = basePWM - correction;
  int pwmRight = basePWM + correction;

  // Ограничиваем значения ШИМ в диапазоне [0..255]
  pwmLeft = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  digitalWrite(LEFT_MOTOR_DIR_PIN, 0);
  analogWrite(LEFT_MOTOR_PWM_PIN, pwmLeft);

  digitalWrite(RIGHT_MOTOR_DIR_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, pwmRight);

  // Для отладки можно выводить значения:
  Serial.print("L: ");
  Serial.print(encoderCountLeft);
  Serial.print("  R: ");
  Serial.print(encoderCountRight);
  Serial.print("  Err: ");
  Serial.print(error);
  Serial.print("  PWM_L: ");
  Serial.print(pwmLeft);
  Serial.print("  PWM_R: ");
  Serial.println(pwmRight);
}
