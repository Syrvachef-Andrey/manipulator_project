import time
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner
import configparser

def move_robot_to(robot, planner, arduino_mcu, current_angles, target_x, target_y, target_z, gripper_angle, steps=50):
    """
    Универсальная функция для перемещения робота.
    Возвращает новые текущие углы после завершения движения.
    """
    print(
        f"-> Едем в точку: X:{target_x * 100:.1f}см, Y:{target_y * 100:.1f}см, Z:{target_z * 100:.1f}см | Захват: {gripper_angle}°")

    # 1. Рассчитываем обратную кинематику для целевой точки
    target_angles = robot.calculate_ik(target_x, target_y, target_z)

    # 2. Генерируем траекторию от текущих углов к целевым
    trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=steps)

    # 3. Отправляем кадры на Arduino
    for frame_angles in trajectory:
        arduino_mcu.send_angles(frame_angles, gripper_angle)
        time.sleep(0.02)  # Пауза 20 мс (50 кадров в секунду)

    return target_angles


def main():
    robot = RobotArm()
    planner = TrajectoryPlanner()

    my_config = configparser.ConfigParser()
    my_config.read('config.ini')

    arduino_mcu = RobotSerial(port=my_config['Arduino']['port'])  # ВАЖНО: укажи свой COM-порт


    time.sleep(2)  # Ждем перезагрузки Arduino

    # Исходное положение робота при включении
    current_angles = [90, 90, 90, 90, 90]

    # ==========================================
    # ТВОЙ СЦЕНАРИЙ ТЕСТИРОВАНИЯ (ПЛЕЙЛИСТ)
    # ==========================================
    # Здесь ты можешь легко добавлять, удалять или менять шаги.
    # x, y, z - координаты в метрах.
    # gripper - угол захвата (например, 45 - открыт, 90 - закрыт).
    # delay - сколько секунд подождать после выполнения этого шага.
    # steps - плавность/скорость движения (больше = медленнее).

    gripper_open = 0
    gripper_close = 90

    test_scenario = [
        # x от -0.3 до 0.3, y от -0.3 до 0.3, z от 0.05 до 0,40
         {"x": 0.15, "y": 0, "z": 0.15, "gripper": gripper_close, "steps": 100, "delay": 1.0, "text": "Положение 1"},
         {"x": -0.15, "y": 0, "z": 0.15, "gripper": gripper_open, "steps": 100, "delay": 1.0, "text": "Положение 2"},
        {"x": 0, "y": 0.00, "z": 0.2, "gripper": gripper_open, "steps": 100, "delay": 1.0, "text": 'Нулевое положение'}
    ]



    # ==========================================
    # ВЫПОЛНЕНИЕ СЦЕНАРИЯ
    # ==========================================
    print("\n--- ЗАПУСК ТЕСТОВОГО СЦЕНАРИЯ ---")

    for i, step in enumerate(test_scenario):
        print(f"\nВыполняю шаг {i + 1} из {len(test_scenario)}...")

        # Вызываем нашу универсальную функцию перемещения
        current_angles = move_robot_to(
            robot=robot,
            planner=planner,
            arduino_mcu=arduino_mcu,
            current_angles=current_angles,
            target_x=step["x"],
            target_y=step["y"],
            target_z=step["z"],
            gripper_angle=step["gripper"],
            steps=step["steps"],
        )

        if step['text'] is not None or not '':
            print(step['text'])

        # Ждем указанное время перед следующим шагом
        if step["delay"] > 0:
            time.sleep(step["delay"])

    print("\n--- СЦЕНАРИЙ УСПЕШНО ЗАВЕРШЕН ---")
    time.sleep(1)
    arduino_mcu.close()


if __name__ == "__main__":
    main()