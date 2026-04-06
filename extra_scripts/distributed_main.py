import time
import configparser
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from kinematics import RobotArm
from communicator import RobotSerial
from trajectory import TrajectoryPlanner


def create_virtual_robot():
    return Chain(name="Virtual_Distributed_Arm", links=[
        OriginLink(),
        URDFLink(
            name="Virt_Base",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            joint_type="revolute"
        ),
        URDFLink(
            name="Virt_Shoulder",
            origin_translation=[0, 0, 0.08],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            joint_type="revolute"
        ),
        URDFLink(
            name="Virt_Elbow",
            origin_translation=[0, 0, 0.18],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            joint_type="revolute"
        ),
        URDFLink(
            name="Virt_Wrist",
            origin_translation=[0, 0, 0.15],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            joint_type="revolute"
        ),
        URDFLink(
            name="Virt_TCP",
            origin_translation=[0, 0, 0.08],
            origin_orientation=[0, 0, 0],
            rotation=None,
            joint_type="fixed"
        )
    ], active_links_mask=[False, True, True, True, True, False])


def move_robot_to(robot, planner, arduino_mcu, current_angles, current_pos, target_x, target_y, target_z, gripper_angle,
                  steps=50, mode="joint"):
    print(f"РЕАЛЬНЫЙ РОБОТ едет в: X:{target_x:.2f}, Y:{target_y:.2f}, Z:{target_z:.2f} Режим: {mode}")

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

    virtual_robot = create_virtual_robot()
    print("Виртуальный робот-двойник успешно инициализирован!")

    current_angles = [90, 90, 90, 90, 90]
    current_pos = [0.0, 0.0, 0.40]
    gripper_close = 90

    test_scenario = [
        {"x": 0.25, "y": 0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 0.5,
         "text": "Приезд на старт"},
        {"x": 0.25, "y": -0.20, "z": 0.15, "mode": "joint", "gripper": gripper_close, "steps": 200, "delay": 0.5,
         "text": "Единая длинная прямая"},
        {"x": 0.00, "y": 0.00, "z": 0.25, "mode": "joint", "gripper": gripper_close, "steps": 100, "delay": 0.5,
         "text": "Нулевое положение"}
    ]


    for i, step in enumerate(test_scenario):
        print(f"ШАГ {i + 1}: {step['text']}")

        move_mode = step.get("mode", "joint")
        target_coords = [step["x"], step["y"], step["z"]]

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

        print(f"ВИРТУАЛЬНЫЙ РОБОТ")
        virt_angles = virtual_robot.inverse_kinematics(target_coords)

        # Настраиваем 3D график
        fig, ax = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(8, 6))
        virtual_robot.plot(virt_angles, ax, target=target_coords)

        ax.set_title(f"Виртуальный робот Шаг {i + 1}\n'Q' на клавиатуре для продолжения", fontsize=12)
        ax.set_xlim(-0.4, 0.4)
        ax.set_ylim(-0.4, 0.4)
        ax.set_zlim(0, 0.5)

        # Обработчик нажатия клавиши 'Q'
        def on_key(event):
            # Проверяем 'q' в английской раскладке и 'й' в русской (частая проблема)
            if event.key in ['q', 'й', 'Q', 'Й']:
                plt.close(event.canvas.figure)

        # Привязываем обработчик к графику
        fig.canvas.mpl_connect('key_press_event', on_key)

        print("'Q' на графике, чтобы продолжить.")

        # plt.show() замораживает выполнение скрипта, пока окно не закроется!
        plt.show()

        # Пауза после закрытия окна, перед следующим шагом
        if step["delay"] > 0:
            time.sleep(step["delay"])

    arduino_mcu.close()


if __name__ == "__main__":
    main()