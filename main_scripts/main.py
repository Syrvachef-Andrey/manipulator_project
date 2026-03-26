import time
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner
import configparser


def move_robot_to(robot, planner, arduino_mcu, current_angles, current_pos, target_x, target_y, target_z, gripper_angle,
                  steps=50, mode="joint"):

    print(f"-> Едем в точку: X:{target_x * 100:.1f}см, Y:{target_y * 100:.1f}см, Z:{target_z * 100:.1f}см | Режим: {mode} | Захват: {gripper_angle}°")

    if target_x < 0:
        print(f"Внимание: опасное значение будущей координаты x - {target_x}, приравнивание её к 0.10")
        target_x = 0
    if target_z < 0.05:
        print(f"Внимание: опасное значение будущей координаты y - {target_y}, приравнивание её к 0.05")
        target_z = 0.05

    end_pos = [target_x, target_y, target_z]

    if mode == "linear":
        trajectory = planner.generate_linear_trajectory(robot, current_angles, current_pos, end_pos, steps)
    else:
        target_angles = robot.calculate_ik(target_x, target_y, target_z)
        trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=steps)

    for frame_angles in trajectory:
        arduino_mcu.send_angles(frame_angles, gripper_angle)
        time.sleep(0.02)


    return trajectory[-1], end_pos


def main():
    robot = RobotArm()
    planner = TrajectoryPlanner()

    my_config = configparser.ConfigParser()
    my_config.read('config.ini')

    arduino_mcu = RobotSerial(port=my_config['Arduino']['port'])

    time.sleep(2)  # Ждем перезагрузки Arduino

    current_angles = [90, 90, 90, 90, 90]
    current_pos = [0.0, 0.0, 0.40]

    gripper_close = 90


    test_scenario = [
        # ШАГ 1: Мягко приезжаем на стартовую точку по дуге (безопасно)
        {"x": 0.25, "y": 0.20, "z": 0.20, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 0.5,
         "text": "Положение 1 (Приезд на старт)"},

        {"x": 0.25, "y": -0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 0.5,
         "text": "Положение 2 (Единая длинная прямая)"},

        # ШАГ 3: Безопасный возврат в центр (строго "joint"!)
        {"x": 0.0, "y": 0.00, "z": 0.30, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 0.5,
         "text": 'Безопасное нулевое положение'}
    ]

    print("\n--- ЗАПУСК ТЕСТОВОГО СЦЕНАРИЯ ---")

    try:
        for i, step in enumerate(test_scenario):
            print(f"\nВыполняю шаг {i + 1} из {len(test_scenario)}...")

            move_mode = step.get("mode", "joint")

            current_angles, current_pos = move_robot_to(
                robot=robot,
                planner=planner,
                arduino_mcu=arduino_mcu,
                current_angles=current_angles,
                current_pos=current_pos,
                target_x=step["x"],
                target_y=step["y"],
                target_z=step["z"],
                gripper_angle=step["gripper"],
                steps=step["steps"],
                mode=move_mode
            )

            if step.get('text'):
                print(step['text'])

            if step["delay"] > 0:
                time.sleep(step["delay"])
    except KeyboardInterrupt:
        print("АВАРИЙНАЯ ОСТАНОВКА СЦЕНАРИЯ")
        print("Безопасное возвращение домой")

        time.sleep(0.5)

        home_scenario = [0.0, 0.0, 0.20]

        home_coord = robot.calculate_ik(home_scenario[0], home_scenario[1], home_scenario[2])

        home_trajectory = planner.generate_joint_trajectory(current_angles, home_coord, steps=100)

        for angles in home_trajectory:
            arduino_mcu.send_angles(angles, gripper_angle=90)
            time.sleep(0.02)
    else:
        print("\n--- СЦЕНАРИЙ УСПЕШНО ЗАВЕРШЕН ---")
    finally:
        time.sleep(1)
        arduino_mcu.close()


if __name__ == "__main__":
    main()