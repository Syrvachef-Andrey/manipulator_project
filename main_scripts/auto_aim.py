import time
import cv2
import numpy as np
import configparser
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner


def get_red_object_error(cap, center_x, center_y):
    """Считывает кадр и возвращает ошибку (err_x, err_y) в пикселях. Если ничего нет, возвращает None"""
    ret, frame = cap.read()
    if not ret:
        return None, None, frame

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 120, 70])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 120, 70])
    upper_red_2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            obj_x = int(M["m10"] / M["m00"])
            obj_y = int(M["m01"] / M["m00"])

            # Отрисовка для наглядности
            cv2.circle(frame, (obj_x, obj_y), 10, (0, 255, 0), -1)
            cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 0, 0), 2)

            return (obj_x - center_x), (obj_y - center_y), frame

    return None, None, frame


def main():
    # --- 1. ИНИЦИАЛИЗАЦИЯ РОБОТА ---
    robot = RobotArm()
    planner = TrajectoryPlanner()
    my_config = configparser.ConfigParser()
    my_config.read('config.ini')
    arduino_mcu = RobotSerial(port=my_config['Arduino']['port'])
    time.sleep(2)

    # --- 2. ИНИЦИАЛИЗАЦИЯ КАМЕРЫ ---
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    height, width, _ = frame.shape
    cam_center_x = int(width / 2)
    cam_center_y = int(height / 2)

    # --- 3. НАСТРОЙКИ НАВЕДЕНИЯ ---
    # Коэффициенты перевода пикселей в метры (возможно, придется сделать один из них с минусом, зависит от того, как прикручена камера)
    Kp_x = 0.00015  # Чувствительность влево/вправо
    Kp_y = 0.00015  # Чувствительность вперед/назад

    DEADZONE = 20  # "Мертвая зона" в пикселях. Если ошибка меньше 20px, считаем, что мы в центре, чтобы робот не дрожал.

    # Стартовая позиция робота (зависаем над столом)
    current_x, current_y, current_z = 0.20, 0.0, 0.20
    current_angles = [90, 90, 90, 90, 90]
    gripper_angle = 90

    print("Выходим на стартовую позицию...")
    target_angles = robot.calculate_ik(current_x, current_y, current_z)
    trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=50)
    for frame_angles in trajectory:
        arduino_mcu.send_angles(frame_angles, gripper_angle)
        time.sleep(0.02)
    current_angles = target_angles

    print("Запуск режима авто-наведения! Покажи красный предмет. Для выхода нажми 'q'.")

    # --- 4. ГЛАВНЫЙ ЦИКЛ ЗРЕНИЯ ---
    while True:
        err_x, err_y, display_frame = get_red_object_error(cap, cam_center_x, cam_center_y)

        # Рисуем перекрестие
        cv2.line(display_frame, (cam_center_x - 20, cam_center_y), (cam_center_x + 20, cam_center_y), (0, 255, 255), 2)
        cv2.line(display_frame, (cam_center_x, cam_center_y - 20), (cam_center_x, cam_center_y + 20), (0, 255, 255), 2)
        cv2.imshow("Robot Vision", display_frame)

        if err_x is not None and err_y is not None:
            # Проверяем, вышли ли мы за пределы мертвой зоны
            if abs(err_x) > DEADZONE or abs(err_y) > DEADZONE:

                # Вычисляем сдвиг в метрах
                # Зависит от ориентации камеры! Если камера смотрит вниз:
                # Ошибка по X на экране -> Движение по Y робота (Влево/Вправо)
                # Ошибка по Y на экране -> Движение по X робота (Вперед/Назад)
                delta_y = err_x * Kp_x
                delta_x = err_y * Kp_y

                # Обновляем целевые координаты
                target_y = current_y - delta_y  # Знак минус или плюс - придется проверить на практике!
                target_x = current_x - delta_x

                # ОГРАНИЧИТЕЛЬ (ЧТОБЫ РОБОТ НЕ УЛЕТЕЛ В СТЕНУ)
                target_x = max(0.10, min(target_x, 0.35))  # X от 10 до 35 см
                target_y = max(-0.25, min(target_y, 0.25))  # Y от -25 до 25 см

                # Считаем IK для нового микро-шага
                target_angles = robot.calculate_ik(target_x, target_y, current_z, initial_angles=current_angles)

                # Делаем быстрый короткий шаг (10 кадров) чтобы робот двигался плавно вслед за объектом
                trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=10)

                for frame_angles in trajectory:
                    arduino_mcu.send_angles(frame_angles, gripper_angle)
                    time.sleep(0.01)

                # Запоминаем новые координаты
                current_angles = target_angles
                current_x, current_y = target_x, target_y

                print(f"Корректировка: X={current_x:.3f}, Y={current_y:.3f}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Завершение
    cap.release()
    cv2.destroyAllWindows()
    arduino_mcu.close()


if __name__ == "__main__":
    main()