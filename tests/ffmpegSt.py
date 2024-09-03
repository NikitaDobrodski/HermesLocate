import subprocess
import cv2
import numpy as np

# Команда для запуска FFmpeg
ffmpeg_command = [
    'ffmpeg',
    '-i', "/home/adam/Documents/HermesLocate/movies/video1bad.mp4",  # Путь к вашему видео
    '-pix_fmt', 'bgr24',
    '-vcodec', 'rawvideo',
    '-an', '-sn',
    '-f', 'image2pipe',
    '-vf', 'scale=480:360',  # Здесь заданы ширина и высота видео
    '-fflags', 'nobuffer',
    '-flags', 'low_delay',
    '-tune', 'zerolatency',
    '-preset', 'ultrafast',
    '-'
]

# Запуск FFmpeg
ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

try:
    while True:
        # Считываем кадр из потока FFmpeg
        raw_frame = ffmpeg_process.stdout.read(480 * 360 * 3)  # 480x360 с 3 каналами (BGR)
        
        if not raw_frame:
            print("Нет данных от FFmpeg, процесс, возможно, завершился.")
            break

        # Преобразуем сырые данные в формат изображения
        frame = np.frombuffer(raw_frame, np.uint8).reshape((360, 480, 3))

        # Отображаем кадр с помощью OpenCV
        cv2.imshow('Video Stream', frame)

        # Прерывание отображения при нажатии 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Ошибка процесса FFmpeg: {e}")
finally:
    if ffmpeg_process.poll() is None:
        ffmpeg_process.terminate()
        ffmpeg_process.wait()

    stderr_output = ffmpeg_process.stderr.read().decode()
    if stderr_output:
        print(f"Ошибка FFmpeg: {stderr_output}")

    # Закрываем все окна OpenCV
    cv2.destroyAllWindows()

    print("Процесс FFmpeg завершен.")
