import qrcode
import cv2


def generate_qr(text: str, filename: str = "my_qr.png"):
    """
    Генерирует QR-код из переданного текста и сохраняет его как изображение.
    """
    # Настройка параметров QR-кода
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )

    qr.add_data(text)
    qr.make(fit=True)

    # Создание и сохранение картинки
    img = qr.make_image(fill_color="black", back_color="white")
    img.save(filename)
    print(f"[УСПЕХ] QR-код с текстом '{text}' сохранен в файл {filename}")


def read_qr_from_camera():
    """
    Открывает веб-камеру, ищет QR-коды в кадре и считывает их текст.
    """
    # Инициализация камеры (0 - стандартная веб-камера)
    cap = cv2.VideoCapture(0)

    # Инициализация встроенного детектора QR-кодов OpenCV
    detector = cv2.QRCodeDetector()

    print("[ИНФО] Камера запущена. Наведите QR-код на камеру.")
    print("[ИНФО] Нажмите клавишу 'q' на английской раскладке для выхода.")

    while True:
        # Чтение кадра с камеры
        ret, frame = cap.read()
        if not ret:
            print("[ОШИБКА] Не удалось получить кадр с камеры.")
            break

        # Распознавание и декодирование QR-кода
        data, bbox, _ = detector.detectAndDecode(frame)

        # Если QR-код найден (есть координаты рамки)
        if bbox is not None:
            # Обводим QR-код зеленой рамкой
            n_lines = len(bbox[0])
            for i in range(n_lines):
                point1 = tuple(map(int, bbox[0][i]))
                point2 = tuple(map(int, bbox[0][(i + 1) % n_lines]))
                cv2.line(frame, point1, point2, color=(0, 255, 0), thickness=2)

            # Если удалось прочитать текст
            if data:
                # Выводим текст в консоль и на экран камеры
                print(f"[РАСПОЗНАНО] Данные: {data}")
                cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Показываем окно с изображением камеры
        cv2.imshow("QR Code Scanner", frame)

        # Выход из цикла при нажатии клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Освобождаем ресурсы
    cap.release()
    cv2.destroyAllWindows()


# --- Блок тестирования ---
if __name__ == "__main__":
    print("Выберите действие:")
    print("1 - Сгенерировать QR-код")
    print("2 - Считать QR-код с камеры")

    choice = input("Введите 1 или 2: ")

    if choice == '1':
        user_text = input("Введите текст или ссылку для QR-кода: ")
        generate_qr(user_text)
    elif choice == '2':
        read_qr_from_camera()
    else:
        print("Неверный ввод.")