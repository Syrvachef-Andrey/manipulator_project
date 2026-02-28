import cv2
import numpy as np


def main():
    # Подключаемся к камере (0 - обычно встроенная веб-камера или первая подключенная по USB)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Ошибка: Камера не найдена!")
        return

    print("Запуск камеры. Для выхода нажми клавишу 'q'.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Узнаем размеры кадра (чтобы найти центр экрана)
        height, width, _ = frame.shape
        center_x = int(width / 2)
        center_y = int(height / 2)

        # Слегка размываем кадр, чтобы убрать мелкий цветовой шум
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        # Переводим кадр из BGR (стандарт OpenCV) в HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Настраиваем диапазоны красного цвета в HSV
        # Красный цвет коварен, он разбит на два куска: в начале шкалы (0-10) и в конце (170-180)
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])

        # Создаем две маски и объединяем их
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # "Подчищаем" маску от мелких точек (морфологические операции)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Ищем контуры (границы) красных объектов
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Если нашли хотя бы один контур
        if len(contours) > 0:
            # Находим самый большой контур (чтобы не отвлекаться на красный фон)
            largest_contour = max(contours, key=cv2.contourArea)

            # Находим моменты контура, чтобы вычислить его центр масс
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                # Координаты центра красного объекта
                obj_x = int(M["m10"] / M["m00"])
                obj_y = int(M["m01"] / M["m00"])

                # Вычисляем ОШИБКУ (насколько объект сдвинут от центра экрана)
                # Если error_x < 0, объект левее центра. Если > 0 - правее.
                error_x = obj_x - center_x
                error_y = obj_y - center_y

                # Рисуем кружок на объекте и линию к центру экрана
                cv2.circle(frame, (obj_x, obj_y), 10, (0, 255, 0), -1)
                cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 0, 0), 2)

                # Выводим данные на экран
                cv2.putText(frame, f"Err X: {error_x} Y: {error_y}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Рисуем перекрестие в центре экрана камеры
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 255), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 255), 2)

        # Показываем результат
        cv2.imshow("Original", frame)
        cv2.imshow("Red Mask", mask)  # Второе окно, чтобы видеть, что именно считает красным

        # Выход по клавише 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()