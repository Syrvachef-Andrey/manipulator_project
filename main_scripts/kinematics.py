import numpy as np
import math
import warnings
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import configparser

class RobotArm:
    def __init__(self):
        self.chain_config = configparser.ConfigParser()
        self.chain_config.read('config.ini')
        # Лимиты для сервоприводов: от -90 до +90 градусов (что равно 0-180 на реальном моторе)
        self.servo_bounds = (math.radians(-90), math.radians(90))

        self.arm = Chain(name="My_5DOF_Arm", links=[
            OriginLink(),
            URDFLink(
                name=self.chain_config['Base']['name'],
                origin_translation=eval(self.chain_config['Base']['origin_translation']),
                origin_orientation=eval(self.chain_config['Base']['origin_orientation']),
                rotation=eval(self.chain_config['Base']['rotation']),
                joint_type=self.chain_config['Base']['joint_type'].strip('"'),
                bounds=self.servo_bounds
            ),
            URDFLink(
                name=self.chain_config['Shoulder']['name'],
                origin_translation=eval(self.chain_config['Shoulder']['origin_translation']),
                origin_orientation=eval(self.chain_config['Shoulder']['origin_orientation']),
                rotation=eval(self.chain_config['Shoulder']['rotation']),
                joint_type=self.chain_config['Shoulder']['joint_type'].strip('"'),
                bounds=self.servo_bounds
            ),
            URDFLink(
                name=self.chain_config['Elbow']['name'],
                origin_translation=eval(self.chain_config['Elbow']['origin_translation']),
                origin_orientation=eval(self.chain_config['Elbow']['origin_orientation']),
                rotation=eval(self.chain_config['Elbow']['rotation']),
                joint_type=self.chain_config['Elbow']['joint_type'].strip('"'),
                bounds=self.servo_bounds
            ),
            URDFLink(
                name=self.chain_config['Forearm']['name'],
                origin_translation=eval(self.chain_config['Forearm']['origin_translation']),
                origin_orientation=eval(self.chain_config['Forearm']['origin_orientation']),
                rotation=eval(self.chain_config['Forearm']['rotation']),
                joint_type=self.chain_config['Forearm']['joint_type'].strip('"'),
                bounds=self.servo_bounds
            ),
            URDFLink(
                name=self.chain_config['Wrist']['name'],
                origin_translation=eval(self.chain_config['Wrist']['origin_translation']),
                origin_orientation=eval(self.chain_config['Wrist']['origin_orientation']),
                rotation=eval(self.chain_config['Wrist']['rotation']),
                joint_type=self.chain_config['Wrist']['joint_type'].strip('"'),
                bounds=self.servo_bounds
            ),
            URDFLink(
                name=self.chain_config['Camera_tcp']['name'],
                origin_translation=eval(self.chain_config['Camera_tcp']['origin_translation']),
                origin_orientation=eval(self.chain_config['Camera_tcp']['origin_orientation']),
                rotation=None,  # None из конфига
                joint_type=self.chain_config['Camera_tcp']['joint_type'].strip('"'),
                bounds=None  # У фиксированного звена нет границ
            )
        ], active_links_mask=eval(self.chain_config['Robot_chains_masks']['chains_mask']))

    def calculate_ik(self, x, y, z, initial_angles=None):
        import math
        import warnings

        target_position = [x, y, z]

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")

            # ВАЖНО: Если переданы предыдущие углы - используем их как подсказку (Anti-Flip)
            if initial_angles is not None:
                # Переводим градусы Ардуино обратно в радианы системы IKPy
                # Добавляем 0 в начало (для Origin) и 0 в конец (для камеры)
                seed = [0] + [math.radians(a - 90) for a in initial_angles] + [0]
                ik_radians = self.arm.inverse_kinematics(target_position, initial_position=seed)
            else:
                # Считаем с нуля (используется по умолчанию для движения по дуге)
                ik_radians = self.arm.inverse_kinematics(target_position)

        # Конвертация в градусы для Arduino (0..180)
        servo_angles = []
        # Нам нужны моторы с индексами от 1 до 5
        for rad in ik_radians[1:6]:
            deg = math.degrees(rad)
            servo_angle = int(round(deg + 90))
            servo_angles.append(servo_angle)

        return servo_angles