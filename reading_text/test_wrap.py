import cv2
import numpy as np
import pytesseract
from ultralytics import YOLO

# 0. Настройка Tesseract (обязательно проверь, совпадает ли путь с твоим)
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

# 1. Настройки путей
model_path = './model/best.pt'
image_path = 'images_for_test_text_tesseract/image.png'

# 2. Загружаем обученную модель
print("Загрузка модели...")
model = YOLO(model_path)

# 3. Читаем изображение
img = cv2.imread(image_path)
if img is None:
    raise ValueError(f"Не удалось найти картинку по пути: {image_path}")

# 4. Прогоняем картинку через нейросеть
print("Поиск ключевых точек...")
results = model(img)

# 5. Извлекаем координаты
if len(results[0].boxes) > 0:
    keypoints = results[0].keypoints.xy[0].cpu().numpy()

    if len(keypoints) == 4:
        pt_TL, pt_TR, pt_BR, pt_BL = keypoints[0], keypoints[1], keypoints[2], keypoints[3]

        pts_src = np.float32([pt_TL, pt_TR, pt_BR, pt_BL])

        # 6. Вычисляем размеры нового "плоского" изображения
        width_A = np.sqrt(((pt_BR[0] - pt_BL[0]) ** 2) + ((pt_BR[1] - pt_BL[1]) ** 2))
        width_B = np.sqrt(((pt_TR[0] - pt_TL[0]) ** 2) + ((pt_TR[1] - pt_TL[1]) ** 2))
        max_width = max(int(width_A), int(width_B))

        height_A = np.sqrt(((pt_TR[0] - pt_BR[0]) ** 2) + ((pt_TR[1] - pt_BR[1]) ** 2))
        height_B = np.sqrt(((pt_TL[0] - pt_BL[0]) ** 2) + ((pt_TL[1] - pt_BL[1]) ** 2))
        max_height = max(int(height_A), int(height_B))

        pts_dst = np.float32([
            [0, 0],
            [max_width - 1, 0],
            [max_width - 1, max_height - 1],
            [0, max_height - 1]
        ])

        # 7. Генерируем матрицу перспективы и "раскатываем" изображение
        matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
        warped_img = cv2.warpPerspective(img, matrix, (max_width, max_height))

        # 8. --- УДАЛЕНИЕ РЖАВЧИНЫ ПО ЦВЕТУ (HSV MASKING) ---
        print("Химическая очистка ржавчины и чтение текста...")

        # 1. Переводим цветную раскатанную картинку в пространство HSV (Оттенок, Насыщенность, Яркость)
        hsv_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

        # 2. Задаем "вилку" цвета ржавчины.
        # Оттенки от красного (0) до желтого (30). Если ржавчина темная, можно снизить пороги.
        lower_rust = np.array([0, 40, 40])
        upper_rust = np.array([30, 255, 255])

        # 3. Ищем все пиксели, попадающие в этот рыже-коричневый диапазон
        rust_mask = cv2.inRange(hsv_img, lower_rust, upper_rust)

        # 4. Копируем исходник и "закрашиваем" найденную ржавчину цветом светлого металла
        clean_warped = warped_img.copy()
        clean_warped[rust_mask > 0] = [200, 200, 200]  # BGR формат (светло-серый)

        # 5. ВОЗВРАЩАЕМСЯ К ТВОЕМУ ЛЮБИМОМУ ФИЛЬТРУ
        gray_img = cv2.cvtColor(clean_warped, cv2.COLOR_BGR2GRAY)

        # Тот самый адаптивный порог, который отработал лучше всего
        binary_image = cv2.adaptiveThreshold(
            gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 5
        )

        # 6. Распознавание
        text = pytesseract.image_to_string(binary_image, lang='eng+rus', config='--psm 11')

        # Вывод окон для контроля
        cv2.imshow("1. Found Rust (Mask)", rust_mask)
        cv2.imshow("2. Cleaned Metal", clean_warped)
        cv2.imshow("3. Your Favorite Binary", binary_image)

        print("\n" + "=" * 40)
        print("📄 РЕЗУЛЬТАТ РАСПОЗНАВАНИЯ:")
        print("=" * 40)
        if text.strip():
            print(text.strip())
        else:
            print("[Пусто]")
        print("=" * 40 + "\n")

        print("Готово! Нажми любую клавишу в окне картинки, чтобы закрыть.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Нейросеть нашла объект, но не смогла определить все 4 точки.")
else:
    print("Нейросеть не нашла шильдик на этой фотографии.")