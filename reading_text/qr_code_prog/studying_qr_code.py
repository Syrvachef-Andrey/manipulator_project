from ultralytics import YOLO

if __name__ == '__main__':
    # Загружаем архитектуру YOLO11 nano для ключевых точек
    model = YOLO('yolo11n.pt')

    print("Запуск двигателей! Начинаем обучение на RTX 4070...")

    # Запускаем тренировку
    results = model.train(
        data='C:\\Users\\Андрей\\PycharmProjects\\manipulator_project\\reading_text\\qr_dataset\\data.yaml',   # Путь к паспорту датасета
        epochs=10,         # Количество проходов по датасету
        imgsz=640,          # Размер, к которому сеть будет приводить картинки
        device=0,           # Указываем использовать главную видеокарту (RTX 4070)
        batch=32,           # По сколько картинок закидывать в видеопамять за раз
        plots=True,         # Построить графики обучения по завершению
        workers=2,          # Количество потоков процессора для подгрузки картинок
        project = 'trained_models',  # Главная папка
        name = 'qr_code_model_v1',  # Подпапка этого эксперимента
    )