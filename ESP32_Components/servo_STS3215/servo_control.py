#!/usr/bin/env python3
import serial
import time

def send_command(ser, cmd):
    """
    Отправляет строковую команду cmd (без \n) в порт,
    затем ждёт ответа одной строкой.
    Возвращает строку ответа (без \n).
    """
    full_cmd = cmd.strip() + "\n"
    ser.write(full_cmd.encode('utf-8'))
    # Считываем одну строку ответа
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    return line

def main():
    # Настройки порта
    port = "COM3"       # или "/dev/ttyUSB0" на Linux
    baudrate = 115200   # Скорость на USB Serial (не путать с 1Mbps к сервам)

    # Открываем порт
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Небольшая пауза, пока Arduino/ESP32 перезагрузится

    # Пример команд:
    # 1) Проверка пинга
    resp = send_command(ser, "PING 1")
    print("PING 1 =>", resp)

    # 2) Чтение текущей позиции (может быть, если серво ID=1)
    resp = send_command(ser, "READ_POS 1")
    print("READ_POS 1 =>", resp)
    # Ожидаем "POS <число>" или "ERROR..."

    # 3) Перемещение серво (ID=1) в позицию 1000 со speed=1500, acc=50
    resp = send_command(ser, "MOVE 1 1000 1500 50")
    print("MOVE =>", resp)
    time.sleep(2)

    # 4) Одновременное перемещение ID=1 и ID=2
    #    SYNC_MOVE <n> <id1> <pos1> <spd1> <acc1> <id2> <pos2> <spd2> <acc2>
    #    например: SYNC_MOVE 2 1 3000 3000 50 2 4095 3400 50
    resp = send_command(ser, "SYNC_MOVE 2 1 3000 3000 50 2 4095 3400 50")
    print("SYNC_MOVE =>", resp)
    time.sleep(2)

    # 5) Чтение температуры на ID=1
    resp = send_command(ser, "READ_TEMP 1")
    print("READ_TEMP 1 =>", resp)

    # 6) (Опционально) Меняем ID сервопривода 1 на 3
    #    resp = send_command(ser, "SET_ID 1 3")
    #    print("SET_ID =>", resp)

    ser.close()

if __name__ == "__main__":
    main()