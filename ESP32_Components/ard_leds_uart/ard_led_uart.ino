#include <Adafruit_NeoPixel.h>

// ------------------------- НАСТРОЙКИ СВЕТОДИОДНОЙ ЛЕНТЫ -------------------------
#define PIN_WS2812B   14     // Пин, к которому подключена лента
#define NUM_PIXELS    50     // Кол-во светодиодов в ленте
#define LED_BRIGHTNESS  50   // Базовая «глобальная» яркость (0-255), если нужно

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // Инициализируем NeoPixel
  ws2812b.begin();
  ws2812b.setBrightness(LED_BRIGHTNESS); // Глобальная яркость (дополнительное масштабирование)
  ws2812b.clear();
  ws2812b.show();

  Serial.println("LED strip + command parser ready.");
}

void loop() {
  // Проверяем наличие данных в Serial
  if (Serial.available()) {
    // Читаем входящую строку до символа '\n'
    String command = Serial.readStringUntil('\n');
    // Удаляем лишние пробелы и перевод строки
    command.trim();
    // Обрабатываем команду
    processCommand(command);
  }

  // Здесь можно делать и другие задачи
  // ...
}

// ----------------------------------------------------------------------------
// Основная функция разбора команды
// ----------------------------------------------------------------------------

void processCommand(String command) {
  // Пример ваших старых команд:
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
  }
  // ----------------- НОВАЯ КОМАНДА ----------------------
  else if (command.startsWith("SET_LED_DATA")) {
    // Парсим формат: SET_LED_DATA <count> <hexString>
    parseSetLedDataCommand(command);
  }
  // ------------------------------------------------------
  else {
    // Если команда не распознана
    Serial.println("ERROR: Unknown command");
  }
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
  int pixelCount  = countStr.toInt();
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
    uint16_t colorVal = (uint16_t) strtol(chunk.c_str(), NULL, 16);

    // Распаковываем (R4, G4, B4, Br)
    uint8_t R4 = (colorVal >> 12) & 0x0F; // старшие 4 бита
    uint8_t G4 = (colorVal >>  8) & 0x0F;
    uint8_t B4 = (colorVal >>  4) & 0x0F;
    uint8_t Br =  colorVal        & 0x0F;

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
    ws2812b.setPixelColor(i, 0); // выключим
  }

  // 4) Обновляем ленту одним вызовом для скорости
  ws2812b.show();

  // Сообщаем об успехе
  Serial.println("OK");
}