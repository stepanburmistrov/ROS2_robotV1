#!/usr/bin/env python3
import serial
import time
import colorsys

def color_to_4bits(r8, g8, b8, brightness):
    """
    Преобразуем 8-битные R,G,B (0..255) и яркость (0..15)
    в 16-битное число с упаковкой: [R(4 бита), G(4 бита), B(4 бита), Br(4 бита)].
    Возвращаем 4-символьную HEX-строку, например "0FAB".
    """
    r4 = (r8 * 15) // 255
    g4 = (g8 * 15) // 255
    b4 = (b8 * 15) // 255
    
    val = (r4 << 12) | (g4 << 8) | (b4 << 4) | (brightness & 0x0F)
    return f"{val:04X}"

def hsv_to_rgb_4bit(h, s, v, brightness):
    """
    Преобразует HSV (0..1) в 4-битный RGB-формат.
    """
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    r8, g8, b8 = int(r * 255), int(g * 255), int(b * 255)
    return color_to_4bits(r8, g8, b8, brightness)

def main():
    port = "COM5"  # Или "/dev/ttyUSB0" на Linux
    baudrate = 115200
    pixel_count = 50
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)
    
    hue = 0.0
    step = 0.01  # Шаг изменения оттенка
    brightness = 3  # Яркость 0-15
    
    try:
        while True:
            data_blocks = [hsv_to_rgb_4bit(hue + (i / pixel_count), 1, 1, brightness) for i in range(pixel_count)]
            hex_data = "".join(data_blocks)
            command = f"SET_LED_DATA {pixel_count} {hex_data}\n"
            print("Sending command:", command)
            ser.write(command.encode('ascii'))
            response = ser.readline().decode('ascii', errors='ignore').strip()
            print("Response:", response)
            
            hue = (hue + step) % 1.0  # Циклическое изменение оттенка
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
        ser.close()

if __name__ == "__main__":
    main()