/***********************************************************************
 *  Пример кода для работы с ICM-20948 (V2) по I2C на Arduino/ESP32
 *  
 *  Соединения (Arduino Uno):
 *    - SDA -> A4
 *    - SCL -> A5
 *    - VCC -> 3.3V или 5V (см. документацию модуля)
 *    - GND -> GND
 *
 *  Соединения (ESP32 DevKit v1):
 *    - SDA -> GPIO 21
 *    - SCL -> GPIO 22
 *    - VCC -> 3.3V
 *    - GND -> GND
 *
 *  (C) 2025 Пример профессионального скрипта
 ***********************************************************************/

#include <Wire.h>

// Адрес IMU по I2C (0x68 при AD0=GND, 0x69 при AD0=VCC)
#define ICM20948_I2C_ADDRESS 0x68

// Регистры
#define REG_BANK_SEL     0x7F // Регистр выбора банка

// ----- Bank 0 -----
#define WHO_AM_I         0x00
#define PWR_MGMT_1       0x06
#define INT_PIN_CFG      0x0F
#define ACCEL_XOUT_H     0x2D
#define ACCEL_XOUT_L     0x2E
#define ACCEL_YOUT_H     0x2F
#define ACCEL_YOUT_L     0x30
#define ACCEL_ZOUT_H     0x31
#define ACCEL_ZOUT_L     0x32
#define GYRO_XOUT_H      0x33
#define GYRO_XOUT_L      0x34
#define GYRO_YOUT_H      0x35
#define GYRO_YOUT_L      0x36
#define GYRO_ZOUT_H      0x37
#define GYRO_ZOUT_L      0x38

// ----- Bank 2 (примеры регистров для настройки акселя/гироскопа) -----
#define GYRO_CONFIG_1    0x01
#define ACCEL_CONFIG     0x14

// Значения по умолчанию WHO_AM_I для ICM-20948
#define WHO_AM_I_VAL     0xEA

// Функция для записи в регистр (в текущем банке)
void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ICM20948_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Функция для чтения одного байта из регистра (в текущем банке)
uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(ICM20948_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(ICM20948_I2C_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF; // Ошибка чтения
}

// Функция для переключения банка регистров
void selectBank(uint8_t bank)
{
  // bank помещается в биты [4:0], поэтому сдвигаем влево на 4 нет необходимости,
  // так как документация ICM-20948 указывает, что биты BNK селектора — [2:0].
  // Но часто встречается описание, где BNK << 4. Проверяйте даташит конкретной ревизии.
  // Для большинства даташитов: BANK = (bank << 4).
  // Однако для ICM-20948 нужно смотреть конкретную реализацию.
  // Допустим, что BNK = (bank << 4) (более типичная реализация).
  uint8_t bankRegVal = (bank << 4);
  writeRegister(REG_BANK_SEL, bankRegVal);
}

// Функция сброса и инициализации ICM-20948
bool initICM20948()
{
  // Сброс модуля - запись 1 в бит DEVICE_RESET регистра PWR_MGMT_1
  selectBank(0);
  writeRegister(PWR_MGMT_1, 0x80); // DEVICE_RESET = 1
  delay(100);

  // Проверяем WHO_AM_I
  uint8_t whoAmI = readRegister(WHO_AM_I);
  if (whoAmI != WHO_AM_I_VAL) {
    Serial.print("Ошибка: WHO_AM_I ожидается 0x");
    Serial.print(WHO_AM_I_VAL, HEX);
    Serial.print(", получено 0x");
    Serial.println(whoAmI, HEX);
    return false; 
  }

  // Выводим сообщение для подтверждения
  Serial.print("ICM-20948 найден, WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);

  // Снимаем модуль из режима сна: очистим бит SLEEP в PWR_MGMT_1
  // И установим автовыбор тактового генератора (кварц/PLL)
  writeRegister(PWR_MGMT_1, 0x01);
  delay(10);

  // Настройка гироскопа и акселерометра
  // Переключаемся в Bank 2 для доступа к GYRO_CONFIG_1, ACCEL_CONFIG и др.
  selectBank(2);

  // GYRO_CONFIG_1:
  // [7:6] = 00 (2000 dps), [5:4] = 00 (фильтр), [3:0] = FCHOICE
  // Для примера установим ±2000 dps, без фильтров (основной режим)
  // Ваша конкретная конфигурация может отличаться
  writeRegister(GYRO_CONFIG_1, 0x00);

  // ACCEL_CONFIG:
  // [7:6] = 00 (±2g), [5:4] = 00 (фильтр), [3:0] = ACCEL_FCHOICE
  // Установим ±2g, без дополнительного фильтра
  writeRegister(ACCEL_CONFIG, 0x00);

  // Возвращаемся в Bank 0 для чтения данных
  selectBank(0);

  return true;
}

// Функция чтения сырого 16-битного значения: два байта (High, Low) в дополнении до 2
int16_t readRawData(uint8_t regHigh, uint8_t regLow)
{
  int16_t highByte = (int16_t)readRegister(regHigh);
  int16_t lowByte  = (int16_t)readRegister(regLow);
  return (int16_t)((highByte << 8) | lowByte);
}

void setup()
{
  Serial.begin(115200);
#if defined(ESP32)
  // Если вы используете нестандартные пины, указывайте их явно
  // Wire.begin(SDA_pin, SCL_pin, 400000); // Частота 400 кГц
  Wire.begin(21, 22, 400000); // ESP32 (по умолчанию) 
#else
  Wire.begin();      // Arduino SDA, SCL
  Wire.setClock(400000); // Частота 400 кГц
#endif

  delay(100);

  Serial.println("Инициализация ICM-20948...");

  if (!initICM20948()) {
    Serial.println("ICM-20948 не обнаружен или ошибка инициализации!");
    while (1) {
      // Бесконечный цикл, если датчик не найден
    }
  }

  Serial.println("ICM-20948 успешно инициализирован.");
}

void loop()
{
  // Считывание «сырых» значений акселерометра
  int16_t accelX = readRawData(ACCEL_XOUT_H, ACCEL_XOUT_L);
  int16_t accelY = readRawData(ACCEL_YOUT_H, ACCEL_YOUT_L);
  int16_t accelZ = readRawData(ACCEL_ZOUT_H, ACCEL_ZOUT_L);

  // Считывание «сырых» значений гироскопа
  int16_t gyroX = readRawData(GYRO_XOUT_H, GYRO_XOUT_L);
  int16_t gyroY = readRawData(GYRO_YOUT_H, GYRO_YOUT_L);
  int16_t gyroZ = readRawData(GYRO_ZOUT_H, GYRO_ZOUT_L);

  // Преобразование в физические величины (примерно)
  // Для выбранного диапазона акселерометра ±2g = 16384 LSB/g
  float aRes = 1.0 / 16384.0;
  float Ax = accelX * aRes;
  float Ay = accelY * aRes;
  float Az = accelZ * aRes;

  // Для гироскопа, если диапазон ±2000 dps, чувствительность ~16.4 LSB/°/s
  float gRes = 1.0 / 16.4;
  float Gx = gyroX * gRes;
  float Gy = gyroY * gRes;
  float Gz = gyroZ * gRes;

  // Вывод в Serial
  Serial.print("Accel (g): X=");
  Serial.print(Ax, 3);
  Serial.print(" Y=");
  Serial.print(Ay, 3);
  Serial.print(" Z=");
  Serial.print(Az, 3);
  Serial.print(" | Gyro (dps): X=");
  Serial.print(Gx, 3);
  Serial.print(" Y=");
  Serial.print(Gy, 3);
  Serial.print(" Z=");
  Serial.println(Gz, 3);

  delay(100); // Задержка для наглядного вывода (~10 Гц)
}