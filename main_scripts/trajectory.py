import math


class TrajectoryPlanner:
    def __init__(self):
        pass

    def generate_joint_trajectory(self, start_angles, target_angles, steps=50):
        """
        Генерирует плавную траекторию между двумя наборами углов.
        Использует косинусоидальную интерполяцию для плавного разгона и торможения (Ease-in / Ease-out).

        :param start_angles: Список текущих углов [j1, j2, j3, j4, j5]
        :param target_angles: Список целевых углов [j1, j2, j3, j4, j5]
        :param steps: Количество промежуточных точек (чем больше, тем плавнее и дольше движение)
        :return: Список массивов углов (кадров движения)
        """
        if len(start_angles) != len(target_angles):
            raise ValueError("Списки стартовых и целевых углов должны быть одинаковой длины")

        trajectory = []

        # Генерируем промежуточные точки
        for i in range(steps + 1):
            # t меняется от 0.0 до 1.0 (линейное время)
            t = i / steps

            # Применяем формулу косинуса для сглаживания:
            # smooth_t будет меняться от 0.0 до 1.0, но нелинейно (медленно в начале и в конце)
            smooth_t = (1 - math.cos(t * math.pi)) / 2.0

            step_angles = []
            for start, target in zip(start_angles, target_angles):
                # Рассчитываем промежуточный угол для каждого сустава
                current_angle = start + (target - start) * smooth_t
                step_angles.append(int(round(current_angle)))

            trajectory.append(step_angles)

        return trajectory