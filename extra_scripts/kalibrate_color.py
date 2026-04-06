import cv2
import numpy as np


def empty(a):
    """Пустая функция, которая нужна для работы ползунков OpenCV"""
    pass


def main():
    print("Запуск калибратора HSV. Нажми 'q' для выхода.")

    # Создаем окно для ползунков
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640, 240)

    # Важно: В OpenCV Hue (Оттенок) измеряется от 0 до 179, а не до 255!
    # Saturation (Насыщенность) и Value (Яркость) измеряются от 0 до 255.

    # Создаем ползунки (Имя, Окно, Стартовое значение, Максимум, Функция-обработчик)
    cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
    cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
    cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
    cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
    cv2.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
    cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

    # Подключаемся к камере
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Ошибка: Камера не найдена!")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Слегка размываем, чтобы убрать шум матрицы камеры
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

        # Переводим кадр в цветовое пространство HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 1. Считываем текущие значения со всех 6 ползунков
        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

        # 2. Формируем массивы границ для OpenCV
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        # 3. Создаем черно-белую маску (всё, что попало в диапазон - белое, остальное - черное)
        mask = cv2.inRange(hsv, lower, upper)

        # 4. Накладываем маску на оригинальный кадр (останется только выделенный цвет)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Добавляем текст прямо на оригинальный кадр, чтобы удобно было списывать значения
        cv2.putText(frame, f"LOWER: [{h_min}, {s_min}, {v_min}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(frame, f"UPPER: [{h_max}, {s_max}, {v_max}]", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Показываем все три окна
        cv2.imshow("Original", frame)
        cv2.imshow("Black/White Mask", mask)
        cv2.imshow("Color Segmented", result)

        # Выход по 'q' или 'й'
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('й'):
            print(f"\nТвои финальные значения для кода:")
            print(f"lower_bound = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper_bound = np.array([{h_max}, {s_max}, {v_max}])\n")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()