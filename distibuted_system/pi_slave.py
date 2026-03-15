import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


def create_virtual_robot():
    return Chain(name="Virtual_Arm", links=[
        OriginLink(),
        URDFLink(name="Virt_Base", origin_translation=[0, 0, 0], origin_orientation=[0, 0, 0], rotation=[0, 0, 1],
                 joint_type="revolute"),
        URDFLink(name="Virt_Shoulder", origin_translation=[0, 0, 0.08], origin_orientation=[0, 0, 0],
                 rotation=[0, 1, 0], joint_type="revolute"),
        URDFLink(name="Virt_Elbow", origin_translation=[0, 0, 0.18], origin_orientation=[0, 0, 0], rotation=[0, 1, 0],
                 joint_type="revolute"),
        URDFLink(name="Virt_Wrist", origin_translation=[0, 0, 0.15], origin_orientation=[0, 0, 0], rotation=[0, 1, 0],
                 joint_type="revolute"),
        URDFLink(name="Virt_TCP", origin_translation=[0, 0, 0.08], origin_orientation=[0, 0, 0], rotation=None,
                 joint_type="fixed")
    ], active_links_mask=[False, True, True, True, True, False])


def main():
    virtual_robot = create_virtual_robot()
    print("Виртуальный робот готов.")

    HOST = '0.0.0.0'
    PORT = 65432

    plt.ion()
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(7, 6))

    current_virt_angles = [0, 0, 0, 0, 0, 0]  # Начальные нули
    virtual_robot.plot(current_virt_angles, ax, target=[0, 0, 0])
    ax.set_xlim(-0.4, 0.4);
    ax.set_ylim(-0.4, 0.4);
    ax.set_zlim(0, 0.5)
    plt.draw()
    plt.pause(0.1)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Ожидаю подключения от ПК на порту {PORT}")

        conn, addr = s.accept()
        with conn:
            print(f"Подключен основной компьютер: {addr}")

            while True:
                data = conn.recv(1024)
                if not data:
                    break

                coords = json.loads(data.decode('utf-8'))
                target = [coords["x"], coords["y"], coords["z"]]
                print(f"Вычисляю IK для цели: {target}")

                target_virt_angles = virtual_robot.inverse_kinematics(target)

                steps = 15
                trajectory_angles = np.linspace(current_virt_angles, target_virt_angles, steps)

                for frame_angles in trajectory_angles:
                    ax.clear()
                    virtual_robot.plot(frame_angles, ax, target=target)
                    ax.set_title(f"Цифровой двойник\nВыполнение команды ПК")
                    ax.set_xlim(-0.4, 0.4);
                    ax.set_ylim(-0.4, 0.4);
                    ax.set_zlim(0, 0.5)

                    plt.draw()
                    plt.pause(0.05)

                current_virt_angles = target_virt_angles

    print("Соединение закрыто.")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()