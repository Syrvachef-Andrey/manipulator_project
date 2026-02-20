import numpy as np
import warnings
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# 1. ОПИСАНИЕ РОБОТА (Исправленная топология URDF)
my_arm = Chain(name="My_5DOF_Arm", links=[
    OriginLink(),

    # Сустав 1: Вращение базы (Z). Сам мотор стоит в самом низу (Z=0).
    URDFLink(
        name="base_pan",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
        joint_type="revolute"
    ),

    # Сустав 2: Наклон плеча (Y). От базы до этого мотора 5 см вверх.
    URDFLink(
        name="shoulder_pitch",
        origin_translation=[0, 0, 0.05],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
        joint_type="revolute"
    ),

    # Сустав 3: Наклон локтя (Y). От мотора плеча до локтя балка 12 см.
    URDFLink(
        name="elbow_pitch",
        origin_translation=[0, 0, 0.12],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
        joint_type="revolute"
    ),

    # Сустав 4: Вращение предплечья (Z). От локтя до вращателя балка 14 см.
    URDFLink(
        name="forearm_roll",
        origin_translation=[0, 0, 0.14],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
        joint_type="revolute"
    ),

    # Сустав 5: Наклон кисти (Y). От вращателя предплечья до кисти балка 7 см.
    URDFLink(
        name="wrist_pitch",
        origin_translation=[0, 0, 0.07],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
        joint_type="revolute"
    ),

    # Фиксированное звено (Гриппер). От кисти до конца инструмента 8.5 см.
    URDFLink(
        name="gripper_tcp",
        origin_translation=[0, 0, 0.085],
        origin_orientation=[0, 0, 0],
        rotation=None,
        joint_type="fixed"
    )
],
               # МАСКА: [Origin, J1, J2, J3, J4, J5, Gripper]
               # True - сустав двигается, False - зафиксирован
               active_links_mask=[False, True, True, True, True, True, False])


# 2. ФУНКЦИЯ ПРОВЕРКИ
def test_kinematics(angles_degrees):
    angles_radians = [0] + [np.deg2rad(a) for a in angles_degrees] + [0]

    # Чтобы ikpy не ругался на то, что мы передаем нули для фиксированных звеньев,
    # мы просто игнорируем предупреждения в момент расчета:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        transformation_matrix = my_arm.forward_kinematics(angles_radians)

    x, y, z = transformation_matrix[:3, 3]

    print("-" * 40)
    print(f"Тестовые углы (град): {angles_degrees}")
    print(f"Координаты TCP:")
    print(f"X: {x * 100:.2f} см")
    print(f"Y: {y * 100:.2f} см")
    print(f"Z: {z * 100:.2f} см")


# 3. ТЕСТОВЫЕ СЦЕНАРИИ
test_kinematics([0, 0, 0, 0, 0])
test_kinematics([0, 90, 0, 0, 0])
test_kinematics([45, 30, -45, 0, 90])