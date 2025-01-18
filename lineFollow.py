#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import serial
import time
import os

# -----------------------------
# Настройки
# -----------------------------
PORT = "/dev/ttyUSB0"    # Порт Arduino 
BAUDRATE = 115200
TIMEOUT = 1.0            # Секунды ожидания ответа

# Коэффициент для «руля»
STEERING_K = 0.008

# Базовая линейная скорость (мм/с)
BASE_LINEAR_SPEED = 200.0

# Порог для выделения «белой/черной линии» в HSV
# Можно корректировать в зависимости от освещения и цвета поля
LOWER_WHITE = (75, 0, 0)
UPPER_WHITE = (111, 149, 175)

# Папка для сохранения кадров
IMAGES_DIR = "images"
# Максимум хранимых фотографий
MAX_IMAGES = 100

# -----------------------------
# Инициализация Serial
# -----------------------------
ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
time.sleep(3)  # Дать время Arduino «подняться», если ресет при открытии порта

def send_command(command):
    """
    Отправляет команду в порт, ждёт ответ от Arduino, печатает его.
    """
    print("[TX] ", command)
    ser.write((command + '\n').encode('utf-8'))

    while True:
        response = ser.readline().decode('utf-8', errors='replace').strip()
        if response:
            print("[RX] ", response)
            break
        else:
            time.sleep(0.05)

# -----------------------------
# Настраиваем камеру
# -----------------------------
cap = cv2.VideoCapture(1)  # Или другой индекс камеры
# При желании можно задать разрешение:
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Ошибка: Камера не найдена!")
    exit(1)

# Убедимся, что папка images/ существует
if not os.path.exists(IMAGES_DIR):
    os.makedirs(IMAGES_DIR)

def main():
    """
    Основная функция: бесконечный цикл чтения кадров,
    определения линии и отправки команд на Arduino.
    Также сохраняем кадры в images/, не более 100 штук, «кольцом».
    """
    last_send_time = time.time()
    imagesSaved = 0  # Счётчик всех кадров (мы используем % MAX_IMAGES для имени)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Ошибка: Не удалось прочитать кадр с камеры.")
                break

            # 1) Для упрощения анализа возьмём нижнюю часть кадра (ROI)
            height, width = frame.shape[:2]
            roi_y1 = int(height * 0.6)
            roi_y2 = height
            roi = frame[roi_y1:roi_y2, 0:width]

            # 2) Преобразуем ROI в HSV для выделения белого
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 3) Маска по диапазону белого
            mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)

            # (Опционально) морфологические операции для сглаживания шумов
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

            # 4) Ищем «центр» белой линии через моменты
            M = cv2.moments(mask, True)
            cx = None
            if M["m00"] > 1000:  # порог площади (подбирайте под вашу линию)
                cx = int(M["m10"] / M["m00"])
            else:
                cx = None

            # Для наглядности сделаем цветную версию маски:
            color_roi = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # Рисуем центральную линию ROI (зелёную)
            cv2.line(color_roi, (width//2, 0), (width//2, roi.shape[0]), (0,255,0), 2)

            if cx is not None:
                # Нарисуем точку, где нашли "центр" линии
                cv2.circle(color_roi, (cx, roi.shape[0]//2), 5, (0,0,255), -1)

            # 5) Считаем ошибку относительно середины ROI => угловая скорость
            if cx is not None:
                error = (cx - (width//2))
                angular_speed = -STEERING_K * error
                linear_speed = BASE_LINEAR_SPEED
            else:
                # Линия потеряна -> остановимся (или можно крутиться, чтобы искать)
                linear_speed = 0.0
                angular_speed = 0.0
                print("[WARN] Белая линия не найдена - останавливаемся!")

            # 6) Отправляем скорость на Arduino (каждые ~0.2с)
            now = time.time()
            if (now - last_send_time) > 0.2:
                cmd = f"SET_VELOCITY {linear_speed:.1f} {angular_speed:.3f}"
                send_command(cmd)
                last_send_time = now

            # 7) Сохраняем кадры (оригинал + разметка) в images/ кольцевым буфером
            idx = imagesSaved % MAX_IMAGES
            cv2.imwrite(os.path.join(IMAGES_DIR, f"orig_{idx:03d}.jpg"), frame)
            cv2.imwrite(os.path.join(IMAGES_DIR, f"annot_{idx:03d}.jpg"), color_roi)
            imagesSaved += 1

            # (Опционально) Можно показать окна, если роботу есть куда выводить:
            # cv2.imshow("Original", frame)
            # cv2.imshow("Mask ROI", color_roi)
            # if cv2.waitKey(1) & 0xFF == 27:  # ESC
            #     break

    finally:
        # Останавливаем робота
        send_command("SET_VELOCITY 0 0")
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Остановка по Ctrl+C")
    finally:
        # Останавливаем робота и закрываем порт
        send_command("SET_VELOCITY 0 0")
        ser.close()
