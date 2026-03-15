import serial
import time


class RobotSerial:
    def __init__(self, port, baudrate=115200):
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            print(f"Подключено к Arduino на порту {port}.")
            # Arduino перезагружается при открытии порта. Ждем 2 секунды.
            time.sleep(2)
        except Exception as e:
            print(f"Ошибка подключения к Arduino: {e}")
            print("Arduino не найдено, выход из программы")
            exit()

    def send_angles(self, angles_list, gripper_angle):
        """
        Отправляет список из 5 углов и угла гриппера в Arduino.
        """
        if not self.arduino:
            print("Нет соединения с Arduino. Данные не отправлены.")
            return

        # Формируем строку: "90,90,90,90,90,45\n"
        # angles_list содержит 5 элементов
        full_command = [str(a) for a in angles_list] + [str(gripper_angle)]
        data_string = ",".join(full_command) + "\n"

        self.arduino.write(data_string.encode('utf-8'))
        # print(f"Отправлено: {data_string.strip()}")

    def close(self):
        if self.arduino:
            self.arduino.close()