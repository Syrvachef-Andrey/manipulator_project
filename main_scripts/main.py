import time
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner
import configparser


def move_robot_to(robot, planner, arduino_mcu, current_angles, current_pos, target_x, target_y, target_z, gripper_angle,
                  steps=50, mode="joint"):
    """
    Универсальная функция для перемещения робота.
    Поддерживает движение по дуге (joint) и по прямой (linear).
    """
    print(
        f"-> Едем в точку: X:{target_x * 100:.1f}см, Y:{target_y * 100:.1f}см, Z:{target_z * 100:.1f}см | Режим: {mode} | Захват: {gripper_angle}°")

    end_pos = [target_x, target_y, target_z]

    if mode == "linear":
        # Движение строго по прямой линии в декартовом пространстве
        trajectory = planner.generate_linear_trajectory(robot, current_angles, current_pos, end_pos, steps)
    else:
        # Рассчитываем обратную кинематику для целевой точки (движение по дуге)
        target_angles = robot.calculate_ik(target_x, target_y, target_z)
        # Генерируем траекторию от текущих углов к целевым
        trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=steps)

    # Отправляем кадры на Arduino
    for frame_angles in trajectory:
        arduino_mcu.send_angles(frame_angles, gripper_angle)
        time.sleep(0.02)  # Пауза 20 мс

    # Возвращаем последние углы траектории и новые координаты
    return trajectory[-1], end_pos


def main():
    robot = RobotArm()
    planner = TrajectoryPlanner()

    my_config = configparser.ConfigParser()
    my_config.read('config.ini')

    arduino_mcu = RobotSerial(port=my_config['Arduino']['port'])

    time.sleep(2)  # Ждем перезагрузки Arduino

    # Исходное положение робота при включении
    current_angles = [90, 90, 90, 90, 90]
    # Примерная позиция, в которой находится робот при углах 90 (ровно вверх)
    current_pos = [0.0, 0.0, 0.40]

    gripper_open = 0
    gripper_close = 90

    # ==========================================
    # ТВОЙ СЦЕНАРИЙ ТЕСТИРОВАНИЯ (ПЛЕЙЛИСТ)
    # ==========================================
    test_scenario = [
        # ШАГ 1: Мягко приезжаем на стартовую точку по дуге (безопасно)
        {"x": 0.25, "y": 0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1.0,
         "text": "Положение 1 (Приезд на старт)"},

        # ШАГ 2: ОДНА длинная непрерывная линия.
        # Увеличили steps до 300, чтобы фильтрам было с чем работать
        {"x": 0.25, "y": -0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1.0,
         "text": "Положение 2 (Единая длинная прямая)"},

        # ШАГ 3: Безопасный возврат в центр (строго "joint"!)
        {"x": 0.0, "y": 0.00, "z": 0.25, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1.0,
         "text": 'Безопасное нулевое положение'}
    ]

    # ==========================================
    # ВЫПОЛНЕНИЕ СЦЕНАРИЯ
    # ==========================================
    print("\n--- ЗАПУСК ТЕСТОВОГО СЦЕНАРИЯ ---")

    for i, step in enumerate(test_scenario):
        print(f"\nВыполняю шаг {i + 1} из {len(test_scenario)}...")

        # Забираем режим из сценария (если не указан, берем "joint" по умолчанию)
        move_mode = step.get("mode", "joint")

        # Вызываем нашу универсальную функцию перемещения (теперь она возвращает 2 значения)
        current_angles, current_pos = move_robot_to(
            robot=robot,
            planner=planner,
            arduino_mcu=arduino_mcu,
            current_angles=current_angles,
            current_pos=current_pos,  # Передаем откуда едем
            target_x=step["x"],
            target_y=step["y"],
            target_z=step["z"],
            gripper_angle=step["gripper"],
            steps=step["steps"],
            mode=move_mode
        )

        if step.get('text'):
            print(step['text'])

        # Ждем указанное время перед следующим шагом
        if step["delay"] > 0:
            time.sleep(step["delay"])

    print("\n--- СЦЕНАРИЙ УСПЕШНО ЗАВЕРШЕН ---")
    time.sleep(1)
    arduino_mcu.close()


if __name__ == "__main__":
    main()