import time
import cv2
import numpy as np
import configparser
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner
from ultralytics import YOLO
from pyzbar.pyzbar import decode


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

        if abs(error) < 40:
            self.integral += error * dt
        else:
            self.integral = 0.0

        self.integral = max(-self.max_integral, min(self.integral, self.max_integral))
        i_term = self.Ki * self.integral

        d_term = 0.0
        if dt > 0:
            d_term = self.Kd * ((error - self.prev_error) / dt)

        self.prev_error = error

        return p_term + i_term + d_term


def get_yolo_qr_data(cap, model, center_x, center_y):
    ret, frame = cap.read()
    if not ret:
        return None, None, None, None, frame

    # 1. YOLO ИНФЕРЕНС (Ищем рамку)
    # verbose=False отключает спам в консоль на каждый кадр
    results = model.predict(frame, conf=0.5, verbose=False)

    best_box = None
    max_area = 0

    # Ищем самый крупный QR-код в кадре
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Получаем координаты углов рамки
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            w = x2 - x1
            h = y2 - y1
            area = w * h

            if area > max_area:
                max_area = area
                best_box = (x1, y1, x2, y2, w, h)

    qr_data = None

    if best_box is not None:
        x1, y1, x2, y2, w, h = best_box
        obj_x = x1 + int(w / 2)
        obj_y = y1 + int(h / 2)

        # 2. РАСПОЗНАВАНИЕ ТЕКСТА (Вырезаем только область с QR-кодом)
        # Добавляем небольшой отступ (padding), чтобы сканеру было проще
        pad = 10
        h_frame, w_frame = frame.shape[:2]
        crop_x1 = max(0, x1 - pad)
        crop_y1 = max(0, y1 - pad)
        crop_x2 = min(w_frame, x2 + pad)
        crop_y2 = min(h_frame, y2 + pad)

        roi = frame[crop_y1:crop_y2, crop_x1:crop_x2]

        # Сканируем вырезанный кусок через pyzbar
        if roi.size > 0:
            decoded_objects = decode(roi)
            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                break  # Берем первый успешно прочитанный

        # --- ОТРИСОВКА ---
        # Если прочитали текст - рамка зеленая, если YOLO видит, но текст не читается - оранжевая
        box_color = (0, 255, 0) if qr_data else (0, 165, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        cv2.circle(frame, (obj_x, obj_y), 5, (0, 0, 255), -1)
        cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 0, 0), 2)

        if qr_data:
            # Выводим прочитанный текст прямо над рамкой в камере
            cv2.putText(frame, qr_data, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        err_x = obj_x - center_x
        err_y = obj_y - center_y

        return err_x, err_y, w, qr_data, frame

    return None, None, None, None, frame


def main():
    robot = RobotArm()
    planner = TrajectoryPlanner()

    my_config = configparser.ConfigParser()
    my_config.read('config.ini')
    arduino_mcu = RobotSerial(port=my_config['Arduino']['port'])
    time.sleep(2)

    # === ЗАГРУЗКА МОДЕЛИ YOLO ===
    # ВСТАВЬ СЮДА ПУТЬ ДО СВОЕЙ ОБУЧЕННОЙ МОДЕЛИ!
    # Например: "runs/detect/train/weights/best.pt"
    print("Загрузка нейросети YOLO...")
    yolo_model = YOLO("yolov8n.pt")
    print("Модель загружена!")

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    height, width, _ = frame.shape
    cam_center_x = int(width / 2)
    cam_center_y = int(height / 2)

    # Наши настроенные коэффициенты ПИД с условием интегратора
    pid_x = PIDController(Kp=0.00003, Ki=0.00001, Kd=0.000001, max_integral=1000)
    pid_y = PIDController(Kp=0.00003, Ki=0.00001, Kd=0.000001, max_integral=1000)
    pid_z = PIDController(Kp=0.00004, Ki=0.000015, Kd=0.00001, max_integral=1500)

    DEADZONE_XY = 5
    DEADZONE_W = 5

    last_time = time.time()

    gripper_angle = 90

    print("\nПлавный вывод робота в рабочую позицию...")

    physical_start_angles = [90, 90, 90, 90, 90]

    ready_x, ready_y, ready_z = 0.25, 0.0, 0.25

    ready_angles = robot.calculate_ik(ready_x, ready_y, ready_z, initial_angles=physical_start_angles)

    startup_trajectory = planner.generate_joint_trajectory(physical_start_angles, ready_angles, steps=100)

    for frame_angles in startup_trajectory:
        arduino_mcu.send_angles(frame_angles, gripper_angle)
        time.sleep(0.02)

    current_angles = ready_angles
    print("Стартовая позиция достигнута!\n")

    TARGET_PIXEL_W = 150

    last_printed_qr = ""

    print("Запуск режима авто-наведения на QR (YOLO)! 'q' для выхода.")

    while True:
        # Теперь получаем данные от YOLO и сканера
        err_x, err_y, pixel_w, qr_text, display_frame = get_yolo_qr_data(cap, yolo_model, cam_center_x, cam_center_y)

        cv2.line(display_frame, (cam_center_x - 20, cam_center_y), (cam_center_x + 20, cam_center_y), (0, 255, 255), 2)
        cv2.line(display_frame, (cam_center_x, cam_center_y - 20), (cam_center_x, cam_center_y + 20), (0, 255, 255), 2)

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Если прочитали НОВЫЙ текст, выводим его в консоль
        if qr_text and qr_text != last_printed_qr:
            print(f"\n[УСПЕХ] Прочитан QR-код: {qr_text}\n")
            last_printed_qr = qr_text

        if err_x is not None and err_y is not None and pixel_w > 0:
            err_w = TARGET_PIXEL_W - pixel_w

            # Проверяем мертвые зоны
            if abs(err_x) > DEADZONE_XY or abs(err_y) > DEADZONE_XY or abs(err_w) > DEADZONE_W:

                # 1. FK
                rad_angles = [0] + [np.deg2rad(a - 90) for a in current_angles] + [0]
                real_pos_matrix = robot.arm.forward_kinematics(rad_angles)
                real_x = real_pos_matrix[0, 3]
                real_y = real_pos_matrix[1, 3]
                real_z = real_pos_matrix[2, 3]

                # 2. ПИД
                delta_x = pid_y.update(err_y, dt)
                delta_y = pid_x.update(err_x, dt)
                delta_z = pid_z.update(err_w, dt)

                target_y = real_y - delta_y
                target_x = real_x + delta_x
                target_z = real_z - delta_z

                # 3. Ограничители
                target_x = max(0.15, min(target_x, 0.35))
                target_y = max(-0.25, min(target_y, 0.25))
                target_z = max(0.10, min(target_z, 0.40))

                # 4. IK
                target_angles = robot.calculate_ik(target_x, target_y, target_z, initial_angles=current_angles)

                # Фильтр скорости
                safe_angles = []
                for curr, targ in zip(current_angles, target_angles):
                    diff = targ - curr
                    if diff > 1:
                        safe_angles.append(curr + 1)
                    elif diff < -1:
                        safe_angles.append(curr - 1)
                    else:
                        safe_angles.append(targ)

                current_angles = safe_angles

            # 5. ОТПРАВКА НА МОТОРЫ
            arduino_mcu.send_angles(current_angles, gripper_angle)

        cv2.imshow("Robot YOLO Vision", display_frame)

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