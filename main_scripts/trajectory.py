import math


class TrajectoryPlanner:
    def __init__(self):
        pass

    def generate_joint_trajectory(self, start_angles, target_angles, steps=50):
        """ДВИЖЕНИЕ ПО ДУГЕ (Остается без изменений, оно работает хорошо)"""
        if len(start_angles) != len(target_angles):
            raise ValueError("Списки стартовых и целевых углов должны быть одинаковой длины")

        trajectory = []
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 1.0
            smooth_t = (1 - math.cos(t * math.pi)) / 2.0

            step_angles = []
            for start, target in zip(start_angles, target_angles):
                current_angle = start + (target - start) * smooth_t
                step_angles.append(int(round(current_angle)))
            trajectory.append(step_angles)
        return trajectory

    def generate_linear_trajectory(self, robot, start_angles, start_pos, end_pos, steps):
        """
        БРОНЕБОЙНОЕ ДВИЖЕНИЕ ПО ПРЯМОЙ С ДВОЙНОЙ ФИЛЬТРАЦИЕЙ
        """
        raw_trajectory = []
        last_angles = start_angles

        # 1. МАКСИМАЛЬНЫЙ СКАЧОК: мотор не может повернуться больше чем на 4 градуса за 1 шаг
        MAX_JUMP = 5.0

        # --- ЭТАП 1: ГЕНЕРАЦИЯ И ЖЕСТКАЯ ОБРЕЗКА (ANTI-FLIP) ---
        for i in range(steps + 1):
            t = i / float(steps) if steps > 0 else 1.0

            # Плавный разгон и торможение вдоль линии
            smooth_t = (1 - math.cos(t * math.pi)) / 2.0

            cx = start_pos[0] + (end_pos[0] - start_pos[0]) * smooth_t
            cy = start_pos[1] + (end_pos[1] - start_pos[1]) * smooth_t
            cz = start_pos[2] + (end_pos[2] - start_pos[2]) * smooth_t

            # Считаем IK с подсказкой предыдущих углов
            target_angles = robot.calculate_ik(cx, cy, cz, initial_angles=last_angles)

            # Проверяем каждый мотор на адекватность
            safe_angles = []
            for prev_a, new_a in zip(last_angles, target_angles):
                diff = new_a - prev_a
                # Если алгоритм сошел с ума и выдал скачок - обрезаем его
                if diff > MAX_JUMP:
                    safe_angles.append(prev_a + MAX_JUMP)
                elif diff < -MAX_JUMP:
                    safe_angles.append(prev_a - MAX_JUMP)
                else:
                    safe_angles.append(new_a)

            raw_trajectory.append(safe_angles)
            last_angles = safe_angles

        # --- ЭТАП 2: СКОЛЬЗЯЩЕЕ СРЕДНЕЕ (СГЛАЖИВАНИЕ ДРОЖИ) ---
        # Усредняем соседние кадры, чтобы траектория стала идеальной
        smoothed_trajectory = []
        window_size = 5  # Размер окна сглаживания (чем больше, тем плавнее, но может чуть скруглить углы)

        for i in range(len(raw_trajectory)):
            # Берем несколько кадров до и после текущего
            start_idx = max(0, i - window_size)
            end_idx = min(len(raw_trajectory), i + window_size + 1)
            window = raw_trajectory[start_idx:end_idx]

            avg_angles = []
            for joint_idx in range(len(start_angles)):
                sum_angles = sum(frame[joint_idx] for frame in window)
                avg_angles.append(int(round(sum_angles / len(window))))

            smoothed_trajectory.append(avg_angles)

        return smoothed_trajectory