import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner
import configparser

# --- ФУНКЦИЯ ВИЗУАЛИЗАЦИИ ---
def visualize_robot(robot, current_angles_deg, target_coords=None):
    if not plt.get_fignums():
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
    else:
        fig = plt.gcf()
        ax = fig.gca()
        ax.cla()

        # Переводим углы для IKPy
    angles_rad = [0] + [np.deg2rad(a - 90) for a in current_angles_deg] + [0]

    # Рисуем
    robot.arm.plot(angles_rad, ax, target=target_coords)

    if target_coords:
        ax.scatter(target_coords[0], target_coords[1], target_coords[2], c='r', marker='o', s=100, label='Target')

    limit = 0.4
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(0, 0.6)

    ax.set_xlabel('X (Вперед)')
    ax.set_ylabel('Y (Влево)')
    ax.set_zlabel('Z (Вверх)')
    ax.legend()
    plt.draw()
    plt.pause(0.01)


# --- ФУНКЦИЯ ДВИЖЕНИЯ (ИСПРАВЛЕНА) ---
def move_robot_to(robot, planner, comms, current_angles, tx, ty, tz, gripper, w_roll, steps):
    # 1. Считаем IK
    # Используем tx, ty, tz которые пришли в аргументах
    target_angles = robot.calculate_ik(tx, ty, tz)

    # 2. Вставляем 5-ю ось
    target_angles[4] = w_roll

    # 3. Траектория
    trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=steps)

    # Отправка
    for frame_angles in trajectory:
        if comms:  # Проверка, подключена ли Arduino
            comms.send_angles(frame_angles, gripper)
        # Маленькая задержка для плавности отрисовки
        time.sleep(0.01)

    return target_angles


# --- MAIN ---
def main():
    robot = RobotArm()
    planner = TrajectoryPlanner()

    plt.ion()

    robot_config = configparser.ConfigParser()

    robot_config.read('config.ini')
    # Подключение к Arduino (или режим симуляции)
    try:
        comms = RobotSerial(port=robot_config['Arduino']['port'])  # <--- ПРОВЕРЬ ПОРТ
        time.sleep(2)
    except:
        print("Arduino не найдено! Работаю только в визуализации.")
        comms = None

    current_angles = [90, 90, 90, 90, 90]
    gripper_open = 0
    gripper_close = 90

    # ТВОЙ СЦЕНАРИЙ
    test_scenario = [
        {"x": 0.35, "y": 0.15, "z": 0.15, "wrist_roll": 90, "gripper": gripper_open, "steps": 200, "delay": 3.0,
         "text": "Положение 1 (Вперед)"},
        {"x": 0.35, "y": 0.15, "z": 0.15, "wrist_roll": 90, "gripper": gripper_close, "steps": 200, "delay": 3.0,
         "text": "Положение 1 (Вперед)"},
        {"x": 0.25, "y": 0.25, "z": 0.15, "wrist_roll": 90, "gripper": gripper_close, "steps": 200, "delay": 3.0,
         "text": "Положение 1 (Вперед)"},
        {"x": 0.25, "y": 0.25, "z": 0.15, "wrist_roll": 90, "gripper": gripper_open, "steps": 200, "delay": 3.0,
         "text": "Положение 1 (Вперед)"},
        # {"x": -0.15, "y": -0.15, "z": 0.15, "wrist_roll": 90, "gripper": gripper_open, "steps": 200, "delay": 5.0,
        #  "text": "Положение 2 (Назад)"},
        {"x": 0, "y": 0.00, "z": 0, "wrist_roll": 90, "gripper": gripper_close, "steps": 200, "delay": 3.0,
         "text": 'Высокое положение'},
        {"x": 0, "y": 0.00, "z": 0, "wrist_roll": 90, "gripper": gripper_open, "steps": 200, "delay": 3.0,
         "text": 'Высокое положение'}
    ]

    print("\n--- ЗАПУСК ВИЗУАЛИЗАЦИИ ---")

    for i, step in enumerate(test_scenario):
        print(f"\n>>> {step['text']}")

        w_roll = step.get("wrist_roll", 90)

        # === ВОТ ЗДЕСЬ БЫЛА ОШИБКА ===
        # Теперь имена аргументов (tx, ty, tz) совпадают с определением функции
        current_angles = move_robot_to(
            robot=robot,
            planner=planner,
            comms=comms,
            current_angles=current_angles,
            tx=step["x"],  # Исправлено target_x -> tx
            ty=step["y"],  # Исправлено target_y -> ty
            tz=step["z"],  # Исправлено target_z -> tz
            gripper=step["gripper"],
            w_roll=w_roll,  # Исправлено wrist_roll -> w_roll
            steps=step["steps"]
        )

        visualize_robot(robot, current_angles, target_coords=[step["x"], step["y"], step["z"]])

        if step["delay"] > 0:
            time.sleep(step["delay"])

    print("\n--- ГОТОВО ---")
    plt.ioff()
    plt.show()

    if comms:
        comms.close()


if __name__ == "__main__":
    main()