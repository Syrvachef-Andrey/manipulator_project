import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
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
# --- ФУНКЦИЯ ДВИЖЕНИЯ (С ПОШАГОВОЙ ОТРИСОВКОЙ) ---
def move_robot_to(robot, planner, comms, current_angles, tx, ty, tz, gripper, w_roll, steps):
    target_angles = robot.calculate_ik(tx, ty, tz)
    target_angles[4] = w_roll
    trajectory = planner.generate_joint_trajectory(current_angles, target_angles, steps=steps)

    target_coords = [tx, ty, tz]  # Сохраняем координаты для красной точки

    # Настройка: как часто обновлять график (каждый 10-й шаг)
    # Если графика все равно тормозит физику - увеличь это число (например, до 15 или 20)
    render_every_n_frames = 10

    for i, frame_angles in enumerate(trajectory):
        # 1. Отправляем углы на реальное железо
        if comms:
            comms.send_angles(frame_angles, gripper)

        # 2. Отрисовка графики
        if i % render_every_n_frames == 0 or i == len(trajectory) - 1:
            visualize_robot(robot, frame_angles, target_coords=target_coords)
        else:
            # Если в этот такт мы не рисуем графику, нужно просто подождать,
            # чтобы сохранить общую скорость движения траектории.
            # (в visualize_robot уже есть пауза 0.01 через plt.pause)
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
    gripper_close = 40
    gripper_open = 0

    test_scenario = [
        # ШАГ 1: Мягко приезжаем на стартовую точку по дуге (безопасно)
        {"x": 0.10, "y": 0.20, "z": 0.05, "mode": "joint", "gripper": gripper_open, "steps": 100, "delay": 1,
         "text": "Положение 1 (Приезд на старт)"},

        {"x": 0.10, "y": 0.0, "z": 0.3, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.10, "y": 0.0, "z": 0.05, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.10, "y": 0.0, "z": 0.05, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.10, "y": 0.0, "z": 0.2, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.20, "y": -0.4, "z": 0.2, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.20, "y": -0.4, "z": 0.05, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.20, "y": -0.4, "z": 0.2, "mode": "joint", "gripper": gripper_open, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"},

        {"x": 0.0, "y": 0.0, "z": 0.4, "mode": "joint", "gripper": gripper_open, "steps": 100, "delay": 1,
         "text": "Положение 2 (Единая длинная прямая)"}
    ]

    print("\n--- ЗАПУСК ВИЗУАЛИЗАЦИИ ---")

    for i, step in enumerate(test_scenario):
        print(f"\n>>> {step['text']}")

        w_roll = step.get("wrist_roll", 90)

        current_angles = move_robot_to(
            robot=robot,
            planner=planner,
            comms=comms,
            current_angles=current_angles,
            tx=step["x"],
            ty=step["y"],
            tz=step["z"],
            gripper=step["gripper"],
            w_roll=w_roll,
            steps=step["steps"]
        )

        # Вызов visualize_robot отсюда УДАЛЕН

        if step["delay"] > 0:
            time.sleep(step["delay"])

    print("\n--- ГОТОВО ---")
    plt.ioff()
    plt.show()

    if comms:
        comms.close()


if __name__ == "__main__":
    main()