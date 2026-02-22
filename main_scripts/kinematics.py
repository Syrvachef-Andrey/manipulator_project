import numpy as np
import math
import warnings
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


class RobotArm:
    def __init__(self):
        # Лимиты для сервоприводов: от -90 до +90 градусов (что равно 0-180 на реальном моторе)
        servo_bounds = (math.radians(-90), math.radians(90))

        self.arm = Chain(name="My_5DOF_Arm", links=[
            OriginLink(),
            URDFLink(name="base", origin_translation=[0, 0, 0], origin_orientation=[0, 0, 0],
                     rotation=[0, 0, 1], joint_type="revolute", bounds=servo_bounds),
            URDFLink(name="shoulder", origin_translation=[0, 0, 0.05], origin_orientation=[0, 0, 0],
                     rotation=[0, 1, 0], joint_type="revolute", bounds=servo_bounds),
            URDFLink(name="elbow", origin_translation=[0, 0, 0.12], origin_orientation=[0, 0, 0],
                     rotation=[0, 1, 0], joint_type="revolute", bounds=servo_bounds),
            URDFLink(name="forearm", origin_translation=[0, 0, 0.14], origin_orientation=[0, 0, 0],
                     rotation=[0, 0, 1], joint_type="revolute", bounds=servo_bounds),
            URDFLink(name="wrist", origin_translation=[0, 0, 0.07], origin_orientation=[0, 0, 0],
                     rotation=[0, 1, 0], joint_type="revolute", bounds=servo_bounds),
            URDFLink(name="gripper_tcp", origin_translation=[0, 0, 0.085], origin_orientation=[0, 0, 0],
                     rotation=None, joint_type="fixed")
        ], active_links_mask=[False, True, True, True, True, True, False])

    def calculate_ik(self, x, y, z):
        """
        Обратная кинематика. Принимает координаты в метрах.
        Возвращает массив из 5 углов (в градусах от 0 до 180) для сервоприводов.
        """
        target_position = [x, y, z]

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            # Считаем IK. optimizer="scalar" работает быстрее и точнее
            ik_radians = self.arm.inverse_kinematics(target_position)

        # Конвертируем радианы в градусы и сдвигаем ноль на 90 градусов
        # ik_radians содержит 7 элементов (включая Origin и TCP). Берем с 1 по 5.
        servo_angles = []
        for rad in ik_radians[1:6]:
            deg = math.degrees(rad)
            # Сдвиг: математический 0 = 90 на сервоприводе
            servo_angle = int(round(deg + 90))
            servo_angles.append(servo_angle)

        return servo_angles