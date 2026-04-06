import json
import os

# Имя твоего скачанного файла
json_file = 'project-3-at-2026-04-06-17-52-c932cc79.json'
output_dir = 'content/labels'

# Создаем папку для текстовых файлов
os.makedirs(output_dir, exist_ok=True)

with open(json_file, 'r', encoding='utf-8') as f:
    data = json.load(f)

saved_files = 0

for item in data:
    # Вытаскиваем оригинальное имя файла (Label Studio добавляет хеш в начало, мы его отрезаем)
    file_upload = item.get('file_upload', '')
    if not file_upload:
        continue
    base_name = file_upload.split('-')[-1].rsplit('.', 1)[0]

    annotations = item.get('annotations', [])
    if not annotations:
        continue

    result = annotations[0].get('result', [])
    if not result:
        continue

    # Собираем координаты 4 точек
    points = {}
    for r in result:
        if r['type'] == 'keypointlabels':
            label = r['value']['keypointlabels'][0]
            # Label Studio выдает проценты (0-100), переводим в доли (0-1)
            x = r['value']['x'] / 100.0
            y = r['value']['y'] / 100.0
            points[label] = (x, y)

    # Если пропущена хоть одна точка - игнорируем кадр
    if len(points) != 4:
        continue

    pt1 = points.get('1_TopLeft')
    pt2 = points.get('2_TopRight')
    pt3 = points.get('3_BottomRight')
    pt4 = points.get('4_BottomLeft')

    if not (pt1 and pt2 and pt3 and pt4):
        continue

    # Вычисляем математический Bounding Box по крайним точкам
    xs = [pt1[0], pt2[0], pt3[0], pt4[0]]
    ys = [pt1[1], pt2[1], pt3[1], pt4[1]]

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    x_center = (xmin + xmax) / 2
    y_center = (ymin + ymax) / 2
    width = xmax - xmin
    height = ymax - ymin

    # Формируем строку формата YOLOv8 Pose
    line = f"0 {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f} " \
           f"{pt1[0]:.6f} {pt1[1]:.6f} 2 " \
           f"{pt2[0]:.6f} {pt2[1]:.6f} 2 " \
           f"{pt3[0]:.6f} {pt3[1]:.6f} 2 " \
           f"{pt4[0]:.6f} {pt4[1]:.6f} 2"

    # Сохраняем в txt файл
    with open(os.path.join(output_dir, f"{base_name}.txt"), 'w') as out_f:
        out_f.write(line)
        saved_files += 1

print(f"Успех! Сгенерировано {saved_files} файлов разметки в папке {output_dir}")