import cv2
import pytesseract
import numpy as np

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"
opened_image = cv2.imread('.\images\image.png')

# 1. Твои координаты с радара
pts_src = np.array([
    [549, 234],
    [1050, 226],
    [1064, 532],
    [543, 512]
], dtype=np.float32)

width, height = 350, 200

pts_dst = np.array([
    [0, 0],
    [width, 0],
    [width, height],
    [0, height]
], dtype=np.float32)

matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
warped_image = cv2.warpPerspective(opened_image, matrix, (width, height))

gray_image = cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY)

binary_image = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 5)

text = pytesseract.image_to_string(binary_image, lang='eng+rus', config='--psm 6')

cv2.imshow("warped image", warped_image)
cv2.imshow("binary image", binary_image)
cv2.imshow("gray_image", gray_image)
cv2.imshow('opened_image', opened_image)

print(text)
cv2.waitKey()
cv2.destroyAllWindows()
