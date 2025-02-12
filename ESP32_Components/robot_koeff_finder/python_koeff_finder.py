import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
import re
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Класс для чтения данных с serial-порта в отдельном потоке
class SerialReader(threading.Thread):
    def __init__(self, serial_port, baudrate, callback):
        super().__init__()
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.callback = callback
        self._stop_event = threading.Event()
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
        except Exception as e:
            print("Error opening serial port:", e)
            self.ser = None

    def run(self):
        if not self.ser:
            return
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.callback(line)
            except Exception as e:
                print("Error reading from serial:", e)
            time.sleep(0.01)

    def stop(self):
        self._stop_event.set()
        if self.ser:
            self.ser.close()

# Главное приложение на Tkinter
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Tuner and Speed Monitor")
        
        # Настройки serial (измените при необходимости)
        self.serial_port = "COM5"  # или "/dev/ttyUSB0"
        self.baudrate = 115200
        self.serial_thread = None
        
        self.create_widgets()
        
        # Настройка Matplotlib для графика
        self.figure = Figure(figsize=(12,4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Wheel Speed Response")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (mm/s)")
        
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().grid(row=7, column=0, columnspan=4, pady=10)
        
        # Списки для накопления данных графика
        self.time_data = []
        self.measured_left_data = []
        self.target_left_data = []
        self.measured_right_data = []
        self.target_right_data = []
        self.start_time = time.time()
        
        self.start_serial()
        self.update_plot()
    
    def create_widgets(self):
        # Ползунки для задания коэффициентов PID
        
        # Kp
        tk.Label(self.root, text="Kp:").grid(row=0, column=0, sticky="e")
        self.kp_scale = tk.Scale(self.root, from_=0, to=10, resolution=0.1,
                                 orient=tk.HORIZONTAL, length=200,
                                 command=self.update_coefficients_from_sliders)
        self.kp_scale.set(2.0)
        self.kp_scale.grid(row=0, column=1)
        
        # Ki
        tk.Label(self.root, text="Ki:").grid(row=0, column=2, sticky="e")
        self.ki_scale = tk.Scale(self.root, from_=0, to=10, resolution=0.1,
                                 orient=tk.HORIZONTAL, length=200,
                                 command=self.update_coefficients_from_sliders)
        self.ki_scale.set(2.5)
        self.ki_scale.grid(row=0, column=3)
        
        # Kd
        tk.Label(self.root, text="Kd:").grid(row=1, column=0, sticky="e")
        self.kd_scale = tk.Scale(self.root, from_=0, to=10, resolution=0.01,
                                 orient=tk.HORIZONTAL, length=200,
                                 command=self.update_coefficients_from_sliders)
        self.kd_scale.set(0.0)
        self.kd_scale.grid(row=1, column=1)
        
        # Kff
        tk.Label(self.root, text="Kff:").grid(row=1, column=2, sticky="e")
        self.kff_scale = tk.Scale(self.root, from_=0, to=1, resolution=0.05,
                                  orient=tk.HORIZONTAL, length=200,
                                  command=self.update_coefficients_from_sliders)
        self.kff_scale.set(0.3)
        self.kff_scale.grid(row=1, column=3)
        
        # Ползунки для задания целевых скоростей колес
        tk.Label(self.root, text="Target Speed Left (mm/s):").grid(row=2, column=0, columnspan=2, sticky="e")
        self.target_left_scale = tk.Scale(self.root, from_=-500, to=500, resolution=1,
                                          orient=tk.HORIZONTAL, length=200,
                                          command=self.update_speed_from_sliders)
        self.target_left_scale.set(0)
        self.target_left_scale.grid(row=2, column=2)
        
        tk.Label(self.root, text="Target Speed Right (mm/s):").grid(row=3, column=0, columnspan=2, sticky="e")
        self.target_right_scale = tk.Scale(self.root, from_=-500, to=500, resolution=1,
                                           orient=tk.HORIZONTAL, length=200,
                                           command=self.update_speed_from_sliders)
        self.target_right_scale.set(0)
        self.target_right_scale.grid(row=3, column=2)
        
        # Текстовое поле для вывода статуса и отладочной информации
        self.status_text = tk.Text(self.root, height=5, width=60)
        self.status_text.grid(row=5, column=0, columnspan=4, pady=5)
    
    def start_serial(self):
        self.serial_thread = SerialReader(self.serial_port, self.baudrate, self.handle_serial_line)
        self.serial_thread.start()
    
    def handle_serial_line(self, line):
        # Вывод полученной строки в текстовом поле
        self.status_text.insert(tk.END, line + "\n")
        self.status_text.see(tk.END)
        # Пробуем распарсить строку с данными скорости.
        # Ожидаемый формат:
        # "POS X=... Y=... Th=... ENC L=... R=... SPD L=123.45 mm/s R=67.89 mm/s Target L=100.00 mm/s R=100.00 mm/s"
        pattern = r"SPD L=([\d\.\-]+) mm/s R=([\d\.\-]+) mm/s.*Target L=([\d\.\-]+) mm/s R=([\d\.\-]+) mm/s"
        match = re.search(pattern, line)
        if match:
            try:
                measured_left = float(match.group(1))
                measured_right = float(match.group(2))
                target_left = float(match.group(3))
                target_right = float(match.group(4))
                current_time = time.time() - self.start_time
                self.time_data.append(current_time)
                self.measured_left_data.append(measured_left)
                self.target_left_data.append(target_left)
                self.measured_right_data.append(measured_right)
                self.target_right_data.append(target_right)
            except Exception as e:
                print("Error parsing speed data:", e)
    
    def update_coefficients_from_sliders(self, value):
        # Формирование команды вида "SET_COEFF Kp Ki Kd Kff" и отправка по serial
        cmd = "SET_COEFF {} {} {} {}\n".format(
            self.kp_scale.get(),
            self.ki_scale.get(),
            self.kd_scale.get(),
            self.kff_scale.get()
        )
        if self.serial_thread and self.serial_thread.ser:
            try:
                self.serial_thread.ser.write(cmd.encode("utf-8"))
                self.status_text.insert(tk.END, "Sent: " + cmd)
                self.status_text.see(tk.END)
            except Exception as e:
                print("Error sending coefficients:", e)
    
    def update_speed_from_sliders(self, value):
        # Формирование команды вида "SET_WHEELS_SPEED leftSpeed rightSpeed" и отправка по serial
        cmd = "SET_WHEELS_SPEED {} {}\n".format(
            self.target_left_scale.get(),
            self.target_right_scale.get()
        )
        if self.serial_thread and self.serial_thread.ser:
            try:
                self.serial_thread.ser.write(cmd.encode("utf-8"))
                self.status_text.insert(tk.END, "Sent: " + cmd)
                self.status_text.see(tk.END)
            except Exception as e:
                print("Error sending wheel speed:", e)
    
    def update_plot(self):
        # Обновление графика с накопленными данными
        self.ax.clear()
        self.ax.set_title("Wheel Speed Response")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (mm/s)")
        self.ax.plot(self.time_data, self.measured_left_data, label="Measured Left")
        self.ax.plot(self.time_data, self.target_left_data, label="Target Left", linestyle="--")
        self.ax.plot(self.time_data, self.measured_right_data, label="Measured Right")
        self.ax.plot(self.time_data, self.target_right_data, label="Target Right", linestyle="--")
        self.ax.legend()
        
        # Определяем окно по 20 секунд
        if self.time_data:
            current_time = self.time_data[-1]
            if current_time < 20:
                self.ax.set_xlim(0, 20)
            else:
                self.ax.set_xlim(current_time - 20, current_time)
        
        self.canvas.draw()
        self.root.after(1000, self.update_plot)
    
    def on_closing(self):
        if self.serial_thread:
            self.serial_thread.stop()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()