import cv2
import os

video_path = "video/shield.mp4"
output_folder = "dataset_frames"
frame_skip = 3

os.makedirs(output_folder, exist_ok=True)

cap = cv2.VideoCapture(video_path)

frame_count = 0
saved_count = 0

print("Начало резки")


while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    if frame_count % frame_skip == 0:
        filename = os.path.join(output_folder, f"frame_{saved_count:04d}.jpg")
        cv2.imwrite(filename, rotated_frame)
        saved_count += 1

    frame_count += 1

cap.release()
print("Окончание")