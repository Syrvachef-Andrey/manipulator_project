import time
import socket
import json
import configparser
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner


PI_IP = '192.168.0.8' # IP Raspberry pi
PORT = 65432 # Порт

def send_to_pi(sock, x, y, z):
    if sock:
        data = json.dumps({"x": x, "y": y, "z": z})
        sock.sendall(data.encode('utf-8'))

def move_robot_to(robot, planner, arduino_mcu, current_angles, current_pos, target_x, target_y, target_z, gripper_angle, steps=50, mode="joint"):
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
    time.sleep(2)

    pi_socket = None
    try:
        pi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        pi_socket.connect((PI_IP, PORT))
        print("Успешно подключение к Raspberry Pi!")
    except Exception as e:
        print(f"Не удалось подключиться к Raspberry Pi по IP {PI_IP}.")
        print("Робот будет работать без визуализации виртуального манипулятора.")
        pi_socket = None

    current_angles = [90, 90, 90, 90, 90]
    current_pos = [0.0, 0.0, 0.40]
    gripper_close = 90

    test_scenario = [
        {"x": 0.25, "y": 0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1.0, "text": "Приезд на старт"},
        {"x": 0.25, "y": -0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 200, "delay": 1.0, "text": "Единая длинная прямая"},
        {"x": 0.10, "y": 0.00, "z": 0.25, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 1.0, "text": "Нулевое положение"}
    ]

    print("\nЗапуск сценария")

    for i, step in enumerate(test_scenario):
        print(f"\nstep {i + 1}: {step['text']}")

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
            mode=step.get("mode", "joint")
        )

        if pi_socket:
            print("Отправка координат на Raspberry Pi")
            send_to_pi(pi_socket, step["x"], step["y"], step["z"])

        if step["delay"] > 0:
            time.sleep(step["delay"])

    print("\nОкончание сценария")
    if pi_socket:
        pi_socket.close()
    arduino_mcu.close()

if __name__ == "__main__":
    main()