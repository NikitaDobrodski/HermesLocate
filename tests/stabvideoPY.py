import os
import cv2
import numpy as np
from vidstab import VidStab
import subprocess

# Инициализация стабилизатора с использованием ORB для обнаружения ключевых точек
stabilizer = VidStab(kp_method='ORB')

# Путь к входному видео
input_path = '/home/adam/Documents/HermesLocate/movies/video1bad.mp4'

# Параметры видео
DESIRED_WIDTH = 480
DESIRED_HEIGHT = 360

# Команда FFmpeg для захвата и декодирования видео
ffmpeg_command = [
    'ffmpeg',  # Убедитесь, что команда начинается с 'ffmpeg'
    '-i', input_path,
    '-pix_fmt', 'bgr24',
    '-vcodec', 'rawvideo',
    '-an', '-sn',
    '-f', 'image2pipe',
    '-vf', f'scale={DESIRED_WIDTH}:{DESIRED_HEIGHT}',
    '-threads', '4',
    '-'
]

ffmpeg_process = None

try:
    # Запуск процесса FFmpeg
    ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)

    while True:
        # Чтение кадра из потока FFmpeg
        raw_frame = ffmpeg_process.stdout.read(DESIRED_WIDTH * DESIRED_HEIGHT * 3)

        if len(raw_frame) != DESIRED_WIDTH * DESIRED_HEIGHT * 3:
            if ffmpeg_process.poll() is not None:
                break
            else:
                break

        # Преобразование сырых данных в изображение OpenCV
        frame = np.frombuffer(raw_frame, np.uint8).reshape((DESIRED_HEIGHT, DESIRED_WIDTH, 3))

        # Стабилизация текущего кадра
        stabilized_frame = stabilizer.stabilize_frame(input_frame=frame, border_type='replicate')

        if stabilized_frame is None:
            stabilized_frame = frame

        # Увеличение размеров отображаемого изображения
        display_width = DESIRED_WIDTH  # Увеличиваем в 2 раза
        display_height = DESIRED_HEIGHT  # Увеличиваем в 2 раза
        resized_original = cv2.resize(frame, (display_width, display_height))
        resized_stabilized = cv2.resize(stabilized_frame, (display_width, display_height))

        # Объединение оригинального и стабилизированного кадров горизонтально
        combined_frame = cv2.hconcat([resized_original, resized_stabilized])

        # Отображение объединенного кадра
        cv2.imshow('Original and Stabilized Frame', combined_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Произошла ошибка: {e}")

finally:
    # Завершение работы с FFmpeg и очистка ресурсов
    if ffmpeg_process:
        ffmpeg_process.stdout.close()
        ffmpeg_process.stderr.close()
        ffmpeg_process.terminate()
        ffmpeg_process.wait()

    cv2.destroyAllWindows()
