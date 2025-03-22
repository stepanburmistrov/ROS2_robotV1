#include <SoftwareSerial.h>

// Пины SoftwareSerial (RX, TX)
#define ESP_RX_PIN 6  // RX Arduino (подключаем к TX GPIO16 ESP32)
#define ESP_TX_PIN 3  // TX Arduino (если нужен обратный канал)

SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

#define NUM_SECTORS 12
int sectorStatusArray[NUM_SECTORS];

// Парсим строку вида "SECTORS: 0,1,2,..."
bool parseSectors(String input, int sectors[]) {
  input.trim();

  if (!input.startsWith("SECTORS: ")) return false;

  input = input.substring(9);  // Убираем префикс "SECTORS: "

  for (int i = 0; i < NUM_SECTORS; i++) {
    int delimiter = input.indexOf(',');

    String token;

    if (delimiter == -1 && i < NUM_SECTORS - 1) return false;  // ошибка формата

    if (delimiter != -1) {
      token = input.substring(0, delimiter);
      input = input.substring(delimiter + 1);
    } else {
      token = input;
    }

    token.trim();
    sectors[i] = token.toInt();
  }

  return true;
}

void setup() {
  Serial.begin(115200);  // монитор порта для отладки
  espSerial.begin(115200); // та же скорость, что и у ESP
  Serial.println("Arduino ready.");
}

void loop() {
  if (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');
    if (parseSectors(line, sectorStatusArray)) {
      Serial.print("Sectors received: ");
      for (int i = 0; i < NUM_SECTORS; i++) {
        Serial.print(sectorStatusArray[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Invalid data format");
    }
  }
}
