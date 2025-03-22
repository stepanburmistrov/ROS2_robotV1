/************************************************************
 * Пример универсального кода для управления
 * FEETECH STS сервоприводами (STS3215 и др.) по Serial1
 * с разбором текстовых команд по Serial (USB).
 *
 * Аппаратные требования:
 *  - ESP32 (GPIO 18 = RX1, GPIO 19 = TX1) или Arduino Mega
 *    (pin 19=RX1, pin 18=TX1) и т. п.
 *  - Скорость Serial1 = 1000000 (1 Мбит/с)
 *  - Библиотека SCServo (например, SMS_STS класс).
 *
 * Поддерживаемые команды (строки без кавычек):
 *  1) PING <id>
 *     -> Проверяет, отвечает ли серво на указанный id
 *     -> Ответ: "OK" или "ERROR"
 *
 *  2) SET_ID <oldId> <newId>
 *     -> Разблокирует EEPROM, меняет id, затем блокирует
 *     -> Ответ: "OK" или "ERROR"
 *
 *  3) MOVE <id> <position> <speed> <acc>
 *     -> Переместить указанный серво к позиции <position>
 *        со speed=<speed>, acc=<acc>.
 *     -> Позиция в пределах 0..4095 (для STS3215)
 *     -> Speed в пределах 0.. ~3400..4095
 *     -> Acc (разгон) ~ 0..255
 *     -> Ответ: "OK"
 *
 *  4) SYNC_MOVE <n> <id1> <pos1> <speed1> <acc1> <id2> <pos2> <speed2> <acc2> ...
 *     -> Одновременно переместить n серв
 *        (макс. n <= 8..12 для удобства, можно расширить).
 *     -> Ответ: "OK"
 *
 *  5) READ_POS <id>
 *     -> Считывает текущую позицию серво
 *     -> Ответ: "POS <значение>" или "ERROR"
 *
 *  6) READ_LOAD <id>
 *     -> Считывает текущую нагрузку серво
 *     -> Ответ: "LOAD <значение>" или "ERROR"
 *
 *  7) READ_VOLTAGE <id>
 *     -> Считывает напряжение питания (мВ или вольты),
 *        в зависимости от реализации библиотеки.
 *     -> Ответ: "VOLTAGE <значение>" или "ERROR"
 *
 *  8) READ_TEMP <id>
 *     -> Считывает температуру серво (°C)
 *     -> Ответ: "TEMP <значение>" или "ERROR"
 *
 *  9) DISABLE_TORQUE <id>
 *     -> Отключает усилие сервопривода
 *     -> Ответ: "OK" или "ERROR"
 *
 ************************************************************/

#include <SCServo.h>

// Класс для управления STS-сервами (FEETECH STS3215 и т.д.)
SMS_STS st;

// Пины UART (приведён пример для ESP32 с UART1, GPIO 18/19).
#define S_RXD 18
#define S_TXD 19

// Скорость UART для серво
#define SERVO_BAUD 1000000

void setup()
{
  // Serial - основная консоль (USB)
  Serial.begin(115200);
  delay(1000);

  // Serial1 - канал для сервоприводов
  Serial1.begin(SERVO_BAUD, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  Serial.println("SCServo universal controller ready.");
}

void loop()
{
  // Ждём команд по основному Serial
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }

  // Можно выполнять другие задачи, если нужно
}

// ------------------------------------------------------
// Обработка входящих команд
// ------------------------------------------------------
void processCommand(String command)
{
  // Разбиваем на токены (по пробелу)
  // Упрощённо - находим первое слово
  int firstSpaceIndex = command.indexOf(' ');
  String cmdName = command;
  String args = "";
  if (firstSpaceIndex > 0)
  {
    cmdName = command.substring(0, firstSpaceIndex);
    args = command.substring(firstSpaceIndex + 1);
    args.trim();
  }

  cmdName.toUpperCase(); // для единообразия

  // Сопоставляем с известными командами
  if (cmdName == "PING")
  {
    handlePing(args);
  }
  else if (cmdName == "SET_ID")
  {
    handleSetID(args);
  }
  else if (cmdName == "MOVE")
  {
    handleMove(args);
  }
  else if (cmdName == "SYNC_MOVE")
  {
    handleSyncMove(args);
  }
  else if (cmdName == "READ_POS")
  {
    handleReadPos(args);
  }
  else if (cmdName == "READ_LOAD")
  {
    handleReadLoad(args);
  }
  else if (cmdName == "READ_VOLTAGE")
  {
    handleReadVoltage(args);
  }
  else if (cmdName == "READ_TEMP")
  {
    handleReadTemp(args);
  } 
  else if (cmdName =="DISABLE_TORQUE")
  {
    handleDisableTorque(args);
  }
  else
  {
    // Неизвестная команда
    Serial.println("ERROR: Unknown command");
  }
}

// ------------------------------------------------------
// Команда: PING <id>
// Проверяем, отвечает ли серво с указанным ID
// ------------------------------------------------------
void handlePing(const String &args)
{
  int id = args.toInt();
  if (id <= 0 || id > 253) // обычный диапазон для ID
  {
    Serial.println("ERROR: Invalid ID");
    return;
  }

  int pingRes = st.Ping(id);
  if (pingRes != -1)
  {
    // Успех
    Serial.println("OK");
  }
  else
  {
    // Ошибка
    Serial.println("ERROR: Ping fail");
  }
}

// ------------------------------------------------------
// Команда: SET_ID <oldId> <newId>
// Меняем ID сервопривода
// ------------------------------------------------------
void handleSetID(const String &args)
{
  // Нужно найти два аргумента (oldId, newId)
  int spaceIdx = args.indexOf(' ');
  if (spaceIdx < 0)
  {
    Serial.println("ERROR: SET_ID usage: SET_ID <oldId> <newId>");
    return;
  }
  String oldIdStr = args.substring(0, spaceIdx);
  String newIdStr = args.substring(spaceIdx + 1);

  int oldId = oldIdStr.toInt();
  int newId = newIdStr.toInt();
  if (oldId <= 0 || oldId > 253 || newId <= 0 || newId > 253)
  {
    Serial.println("ERROR: Invalid ID range");
    return;
  }

  // Разблокируем EEPROM
  st.unLockEprom(oldId);
  // Пытаемся записать новый ID
  // (адрес параметра ID - SMS_STS_ID = 0x05, но библиотека имеет helper: writeByte)
  bool writeOk = st.writeByte(oldId, SMS_STS_ID, newId);
  // Блокируем EEPROM
  st.LockEprom(newId);

  if (writeOk)
    Serial.println("OK");
  else
    Serial.println("ERROR: write ID fail");
}

// ------------------------------------------------------
// Команда: MOVE <id> <position> <speed> <acc>
// ------------------------------------------------------
void handleMove(const String &args)
{
  // Нужно 4 аргумента
  int sp1 = args.indexOf(' ');
  if (sp1 < 0) { Serial.println("ERROR: invalid MOVE usage"); return; }
  int sp2 = args.indexOf(' ', sp1 + 1);
  if (sp2 < 0) { Serial.println("ERROR: invalid MOVE usage"); return; }
  int sp3 = args.indexOf(' ', sp2 + 1);
  if (sp3 < 0) { Serial.println("ERROR: invalid MOVE usage"); return; }

  String idStr  = args.substring(0, sp1);
  String posStr = args.substring(sp1 + 1, sp2);
  String spdStr = args.substring(sp2 + 1, sp3);
  String accStr = args.substring(sp3 + 1);

  int id    = idStr.toInt();
  int pos   = posStr.toInt();
  int speed = spdStr.toInt();
  int acc   = accStr.toInt();

  // Можно добавить проверки на диапазоны
  if (id <= 0 || id > 253) { Serial.println("ERROR: invalid ID"); return; }
  // Позиция STS3215: 0..4095, скорость 0..4095, acc 0..255
  // (точные ограничения см. в даташите)
  if (pos < 0) pos = 0;
  if (pos > 4095) pos = 4095;
  if (speed < 0) speed = 0;
  if (speed > 4095) speed = 4095;
  if (acc < 0) acc = 0;
  if (acc > 255) acc = 255;

  // Выполняем перемещение
  st.WritePosEx(id, pos, speed, acc);
  Serial.println("OK");
}

// ------------------------------------------------------
// Команда: SYNC_MOVE <n> <id1> <pos1> <speed1> <acc1> ...
// Пример: SYNC_MOVE 2 1 3000 3400 50 2 2000 2000 50
// ------------------------------------------------------
void handleSyncMove(const String &args)
{
  // Считываем <n>, затем n групп по 4 числа
  int sp1 = args.indexOf(' ');
  if (sp1 < 0) {
    Serial.println("ERROR: invalid SYNC_MOVE usage");
    return;
  }

  String nStr = args.substring(0, sp1);
  int n = nStr.toInt();
  if (n < 1 || n > 12) {
    Serial.println("ERROR: n out of range");
    return;
  }

  // Массивы для SyncWritePosEx
  byte  IDs[12];
  s16   Positions[12];
  u16   Speeds[12];
  byte  Accs[12];

  // Двигаемся по остальным токенам
  String remain = args.substring(sp1 + 1);
  remain.trim();

  // Нам нужно n*4 чисел
  // Пошагово парсим
  for (int i = 0; i < n; i++) {
    // каждая группа: <id> <pos> <speed> <acc>
    // находим 3 пробела
    // remain = "id pos speed acc [остальное]"
    int spA = remain.indexOf(' ');
    if (spA < 0) { Serial.println("ERROR: not enough arguments for SYNC_MOVE"); return; }
    int spB = remain.indexOf(' ', spA + 1);
    if (spB < 0) { Serial.println("ERROR: not enough arguments for SYNC_MOVE"); return; }
    int spC = remain.indexOf(' ', spB + 1);
    if (spC < 0) { Serial.println("ERROR: not enough arguments for SYNC_MOVE"); return; }

    // Извлекаем подстроки
    String idStr  = remain.substring(0, spA);
    String posStr = remain.substring(spA + 1, spB);
    String spdStr = remain.substring(spB + 1, spC);

    // Следующее - или до конца строки, или след. пробел
    // но нам нужно точно acc. Его ищем как остаток до следующего пробела (или конца строки)
    int spNext = remain.indexOf(' ', spC + 1);
    String accStr;
    if (spNext >= 0) {
      accStr = remain.substring(spC + 1, spNext);
      remain = remain.substring(spNext + 1);
    } else {
      // Это последняя группа
      accStr = remain.substring(spC + 1);
      remain = "";
    }
    remain.trim();

    // Конвертим
    int id    = idStr.toInt();
    int pos   = posStr.toInt();
    int speed = spdStr.toInt();
    int acc   = accStr.toInt();

    // Проверки, обрезаем диапазоны
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;
    if (speed < 0) speed = 0;
    if (speed > 4095) speed = 4095;
    if (acc < 0) acc = 0;
    if (acc > 255) acc = 255;

    IDs[i]       = (byte)id;
    Positions[i] = (s16)pos;
    Speeds[i]    = (u16)speed;
    Accs[i]      = (byte)acc;
  }

  // Всё распарсили, вызываем SyncWrite
  st.SyncWritePosEx(IDs, n, Positions, Speeds, Accs);
  Serial.println("OK");
}

// ------------------------------------------------------
// Чтение текущей позиции
// Команда: READ_POS <id>
// Ответ: "POS <value>" или "ERROR"
// ------------------------------------------------------
void handleReadPos(const String &args)
{
  int id = args.toInt();
  if (id <= 0 || id > 253) {
    Serial.println("ERROR: invalid ID");
    return;
  }
  int pos = st.ReadPos(id);
  if (pos != -1) {
    Serial.print("POS ");
    Serial.println(pos);
  } else {
    Serial.println("ERROR: read pos fail");
  }
}

// ------------------------------------------------------
// Чтение нагрузки
// Команда: READ_LOAD <id>
// Ответ: "LOAD <value>" или "ERROR"
// ------------------------------------------------------
void handleReadLoad(const String &args)
{
  int id = args.toInt();
  if (id <= 0 || id > 253) {
    Serial.println("ERROR: invalid ID");
    return;
  }
  int load = st.ReadLoad(id);
  if (load != -1) {
    // Для STS: значение может быть от 0 до ~2047, знак нагрузки
    // Некоторыми библиотеками нагрузка может храниться в <0..+2047> (бит старшего знака)
    // Нужно смотреть реализацию SCServo. Часто бит 10 = направление нагрузки (тянет/давит).
    Serial.print("LOAD ");
    Serial.println(load);
  } else {
    Serial.println("ERROR: read load fail");
  }
}

// ------------------------------------------------------
// Чтение напряжения
// Команда: READ_VOLTAGE <id>
// Ответ: "VOLTAGE <value>" или "ERROR"
// Обычно в милливольтах, либо десятых вольта
// ------------------------------------------------------
void handleReadVoltage(const String &args)
{
  int id = args.toInt();
  if (id <= 0 || id > 253) {
    Serial.println("ERROR: invalid ID");
    return;
  }
  int volt = st.ReadVoltage(id);
  if (volt != -1) {
    // В разных версиях библиотеки это может быть 0..200 (дес. вольты) или 0..255
    // Допустим, в SCServo: "voltage = readByte(id, SMS_STS_Present_Voltage);" (0..255),
    // иногда интерпретируется как 0.1В, т.е. 50 => 5.0В. Но зависит от реализации.
    Serial.print("VOLTAGE ");
    Serial.println(volt);
  } else {
    Serial.println("ERROR: read voltage fail");
  }
}

// ------------------------------------------------------
// Чтение температуры
// Команда: READ_TEMP <id>
// Ответ: "TEMP <value>" или "ERROR"
// ------------------------------------------------------
void handleReadTemp(const String &args)
{
  int id = args.toInt();
  if (id <= 0 || id > 253) {
    Serial.println("ERROR: invalid ID");
    return;
  }
  int temp = st.ReadTemper(id);
  if (temp != -1) {
    Serial.print("TEMP ");
    Serial.println(temp);
  } else {
    Serial.println("ERROR: read temperature fail");
  }
}

// ------------------------------------------------------
// Отключение усилия сервопривода
// Команда: DISABLE_TORQUE <id>
// Ответ: "OK" или "ERROR"
// ------------------------------------------------------
void handleDisableTorque(const String &args)
{
  int id = args.toInt();
  if (id <= 0 || id > 253) // обычный диапазон для ID
  {
    Serial.println("ERROR: Invalid ID");
    return;
  }

  int pingRes = st.EnableTorque(id, 0);
  if (pingRes != -1)
  {
    // Успех
    Serial.println("OK");
  }
  else
  {
    // Ошибка
    Serial.println("ERROR: disable torque fail");
  }
}