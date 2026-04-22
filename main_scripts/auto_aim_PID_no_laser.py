import time
import cv2
import numpy as np
import configparser
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_integral=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0.0
        self.prev_error = 0.0

        self.max_integral = max_integral

    def update(self, error, dt):
        p_term = self.Kp * error

        self.integral += error * dt

        self.integral = max(-self.max_integral, min(self.integral, self.max_integral))
        i_term = self.Ki * self.integral

        d_term = 0.0
        if dt > 0:
            d_term = self.Kd * ((error - self.prev_error) / dt)

        self.prev_error = error

        return p_term + i_term + d_term

def get_green_object_data(cap, center_x, center_y):
    ret, frame = cap.read()
    if not ret:
        return None, None, None, frame

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower_green = np.array([40, 50, 50])
    upper_green = np.array([90, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cv2.imshow('Black-White Mask', mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        # Возвращаем стандартный порог площади
        if cv2.contourArea(largest_contour) > 1000:
            # Bounding Box для ширины объекта
            x, y, w, h = cv2.boundingRect(largest_contour)

            obj_x = x + int(w / 2)
            obj_y = y + int(h / 2)

            # Отрисовка
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (obj_x, obj_y), 5, (0, 0, 255), -1)
            cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 0, 0), 2)

            err_x = obj_x - center_x
            err_y = obj_y - center_y

            return err_x, err_y, w, frame

    return None, None, None, frame


def main():
    robot = RobotArm()
    planner = TrajectoryPlanner()

    my_config = configparser.ConfigParser()
    my_config.read('config.ini')
    arduino_mcu = RobotSerial(port=my_config['Arduino']['port'])
    time.sleep(2)

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    height, width, _ = frame.shape
    cam_center_x = int(width / 2)
    cam_center_y = int(height / 2)

    # Стартовые безопасные коэффициенты ПИД
    pid_x = PIDController(Kp=0.00003, Ki=0.000025, Kd=0.000001, max_integral=3000)
    pid_y = PIDController(Kp=0.00003, Ki=0.000025, Kd=0.000001, max_integral=3000)

    # Для Z тоже нужен интеграл, чтобы Error W доходил до нуля
    pid_z = PIDController(Kp=0.00004, Ki=0.00001, Kd=0.00001, max_integral=3000)

    DEADZONE_XY = 1
    DEADZONE_W = 3

    last_time = time.time()

    current_angles = [90, 90, 90, 90, 90]
    gripper_angle = 90

    # Желаемая ширина кубика (в пикселях)
    TARGET_PIXEL_W = 200

    print("Запуск режима авто-наведения БЕЗ ЛАЗЕРА (Базовый ПИД)! Покажи темно-зеленый куб. 'q' для выхода.")

    while True:
        err_x, err_y, pixel_w, display_frame = get_green_object_data(cap, cam_center_x, cam_center_y)

        cv2.line(display_frame, (cam_center_x - 20, cam_center_y), (cam_center_x + 20, cam_center_y), (0, 255, 255), 2)
        cv2.line(display_frame, (cam_center_x, cam_center_y - 20), (cam_center_x, cam_center_y + 20), (0, 255, 255), 2)

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        if err_x is not None and err_y is not None and pixel_w > 0:
            # Ошибка дистанции
            err_w = TARGET_PIXEL_W - pixel_w

            cv2.putText(display_frame, f"Width: {pixel_w} px", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Проверяем мертвые зоны по сырому сигналу
            if abs(err_x) > DEADZONE_XY or abs(err_y) > DEADZONE_XY or abs(err_w) > DEADZONE_W:

                # 1. FK для реальной позиции
                rad_angles = [0] + [np.deg2rad(a - 90) for a in current_angles] + [0]
                real_pos_matrix = robot.arm.forward_kinematics(rad_angles)
                real_x = real_pos_matrix[0, 3]
                real_y = real_pos_matrix[1, 3]
                real_z = real_pos_matrix[2, 3]

                # 2. === ПИД ===
                # Кормим ПИД сырыми данными с камеры
                delta_x = pid_y.update(err_y, dt)
                delta_y = pid_x.update(err_x, dt)
                delta_z = pid_z.update(err_w, dt)

                target_y = real_y - delta_y
                target_x = real_x + delta_x
                target_z = real_z - delta_z

                # 3. Ограничители рабочей зоны (с исправленным X)
                target_x = max(0.15, min(target_x, 0.35))
                target_y = max(-0.25, min(target_y, 0.25))
                target_z = max(0.10, min(target_z, 0.40))

                # 4. IK и вычисление шага
                target_angles = robot.calculate_ik(target_x, target_y, target_z, initial_angles=current_angles)

                # Твой фильтр безопасности скорости (1 градус/кадр)
                safe_angles = []
                for curr, targ in zip(current_angles, target_angles):
                    diff = targ - curr
                    if diff > 1:
                        safe_angles.append(curr + 1)
                    elif diff < -1:
                        safe_angles.append(curr - 1)
                    else:
                        safe_angles.append(targ)

                print(f"Target: X={target_x:.2f} Y={target_y:.2f} Z={target_z:.2f} | Error W: {err_w}")
                current_angles = safe_angles

            # 5. ОТПРАВКА
            arduino_mcu.send_angles(current_angles, gripper_angle)

        cv2.imshow("Robot 3D Vision", display_frame)

        # Выход по клавише 'q' (или русской 'й')
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('й'):
            break

    # --- ЗАКРЫВАЕМ ОКНА КАМЕРЫ ---
    cap.release()
    cv2.destroyAllWindows()
    print("\nКамера отключена.")

    # --- БЕЗОПАСНАЯ ПАРКОВКА ---
    print("Возврат в безопасное положение (X: 0, Y: 0, Z: 0.2)...")
    target_x, target_y, target_z = 0.0, 0.0, 0.20

    home_angles = robot.calculate_ik(target_x, target_y, target_z)
    trajectory = planner.generate_joint_trajectory(current_angles, home_angles, steps=100)

    for frame_angles in trajectory:
        arduino_mcu.send_angles(frame_angles, gripper_angle)
        time.sleep(0.02)

    print("Робот успешно припаркован. Завершение работы.")
    arduino_mcu.close()

if __name__ == "__main__":
    main()