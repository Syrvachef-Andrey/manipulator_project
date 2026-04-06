import cv2

# Функция, которая срабатывает при клике мышки
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Печатаем координаты в консоль
        print(f"[{x}, {y}],")
        # Рисуем красную точку там, где ты кликнул, чтобы не запутаться
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('Click 4 corners', img)

# Загружаем твою картинку
img = cv2.imread('.\images\image.png')

cv2.imshow('Click 4 corners', img)
# Привязываем нашу функцию клика к окну
cv2.setMouseCallback('Click 4 corners', click_event)

print("Кликни мышкой по 4 углам шильдика в таком порядке:")
print("1. Левый верхний")
print("2. Правый верхний")
print("3. Правый нижний")
print("4. Левый нижний")
print("После кликов нажми любую клавишу для выхода.")

cv2.waitKey(0)
cv2.destroyAllWindows()