"""main_video.py"""

#!/usr/bin/python3

import cv2
import queue
import time
import numpy as np
import logging
from threading import Thread, Event, Lock
from pymavlink import mavutil
import subprocess
import os
import signal
import sys
from socket_server import SocketServer
from support_positioning import SupportPositioning

# Set the Qt platform to avoid missing plugin error
os.environ["QT_QPA_PLATFORM"] = "xcb"

# Функция для поворота точки
def rotate_point(x, y, angle_degrees):
    from math import radians, cos, sin
    angle_radians = radians(angle_degrees)
    x_new = x * cos(angle_radians) - y * sin(angle_radians)
    y_new = x * sin(angle_radians) + y * cos(angle_radians)
    return x_new, y_new

# Функция стабилизации видео
def stabilize_video(video_source, stop_event, lock):
    global stabilized_frame
    cap = cv2.VideoCapture(video_source)
    
    if not cap.isOpened():
        print("Ошибка открытия видеопотока")
        return

    ret, prev_frame = cap.read()
    if not ret:
        print("Не удалось считать кадр")
        return

    scale_factor = 0.5
    width = int(prev_frame.shape[1] * scale_factor)
    height = int(prev_frame.shape[0] * scale_factor)
    dim = (width, height)

    prev_frame = cv2.resize(prev_frame, dim, interpolation=cv2.INTER_LINEAR)
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(prev_gray, None)

    kalman = cv2.KalmanFilter(6, 3)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                         [0, 1, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 0, 1, 0, 0],
                                        [0, 1, 0, 0, 1, 0],
                                        [0, 0, 1, 0, 0, 1],
                                        [0, 0, 0, 1, 0, 0],
                                        [0, 0, 0, 0, 1, 0],
                                        [0, 0, 0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 0.03
    kalman.statePre = np.zeros(6, dtype=np.float32)
    kalman.statePost = np.zeros(6, dtype=np.float32)

    frame_count = 0
    skip_frames = 2
    stabilized_frame = None

    def display_thread():
        global stabilized_frame
        while not stop_event.is_set():
            with lock:
                if stabilized_frame is not None:
                    cv2.imshow("Stabilized Video", stabilized_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()

    display_thread = Thread(target=display_thread)
    display_thread.start()

    while not stop_event.is_set():
        ret, curr_frame = cap.read()
        if not ret:
            break

        frame_count += 1
        if frame_count % skip_frames == 0:
            continue

        curr_frame = cv2.resize(curr_frame, dim, interpolation=cv2.INTER_LINEAR)
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        kp2, des2 = orb.detectAndCompute(curr_gray, None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        if len(matches) >= 4:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            M, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
            if M is not None:
                M = np.vstack([M, [0, 0, 1]])
                kalman.correct(np.array([M[0, 2], M[1, 2], np.arctan2(M[1, 0], M[0, 0])], np.float32))
                predicted = kalman.predict()

                angle = predicted[2]
                dx = predicted[0]
                dy = predicted[1]
                smoothed_M = np.array([[np.cos(angle), -np.sin(angle), dx],
                                       [np.sin(angle), np.cos(angle), dy],
                                       [0, 0, 1]], dtype=np.float32)

                with lock:
                    stabilized_frame = cv2.warpPerspective(curr_frame, smoothed_M, (curr_frame.shape[1], curr_frame.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
            else:
                print("Не удалось вычислить матрицу преобразования")
                with lock:
                    stabilized_frame = curr_frame
        else:
            print(f"Недостаточно точек для вычисления преобразования: {len(matches)}")
            with lock:
                stabilized_frame = curr_frame

        prev_gray = curr_gray.copy()
        kp1, des1 = orb.detectAndCompute(prev_gray, None)

    stop_event.set()
    display_thread.join()
    cap.release()
    cv2.destroyAllWindows()

# Функция обработки сигналов
def signal_handler(sig, frame):
    logging.info('Программа прервана пользователем!')
    stop_event.set()

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Директория для логов
log_dir = '/home/adam/Documents/HermesLocate/logs'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# Настройка логирования
logging.basicConfig(filename=os.path.join(log_dir, 'flight_log.log'), level=logging.INFO, format='%(asctime)s - %(message)s')

# Параметры видео
VIDEO_PATH = "/home/adam/Documents/HermesLocate/movies/video1bad.mp4"
DESIRED_WIDTH = 480
DESIRED_HEIGHT = 360
COORDINATE_INTERVAL = 1
FPS = 30

# Настройка и запуск сокет-сервера
socket_server = SocketServer('192.168.144.5', 5008)

def start_server():
    """Запуск сервера сокетов."""
    socket_server.start()

# Запуск сокет-сервера в отдельном потоке
server_thread = Thread(target=start_server, daemon=True)
server_thread.start()

class VideoProcessor:
    def __init__(self, video_path):
        self.video_path = video_path
        self.stop_event = Event()
        self.frame_queue = queue.Queue(maxsize=10)
        self.thread_read = None
        self.thread_display = None
        self.master = self.initialize_mavlink()
        self.initial_coordinates = self.get_initial_coordinates()
        self.support_positioning = SupportPositioning(self.master)
        self.processed_frames = 0
        self.lock = Lock()

    def initialize_mavlink(self):
        try:
            master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
            if not master:
                logging.error("Не удалось установить соединение MAVLink")
                raise ValueError("Не удалось установить соединение MAVLink")
            return master
        except Exception as e:
            logging.error(f"Ошибка инициализации соединения MAVLink: {e}")
            sys.exit(1)

    def get_initial_coordinates(self):
        logging.info("Ожидание получения начальных координат от клиента...")
        while True:
            data = socket_server.receive_data()
            if data:
                try:
                    x_start, y_start = map(float, data.split(','))
                    logging.info(f"Получены начальные координаты от клиента: X={x_start}, Y={y_start}")
                    return x_start, y_start
                except ValueError:
                    logging.error("Не удалось распарсить данные координат. Ожидание новых данных...")

    def read_frames(self):
        try:
            ffmpeg_command = [
                'ffmpeg',
                '-i', self.video_path,
                '-pix_fmt', 'bgr24',
                '-vcodec', 'rawvideo',
                '-an', '-sn',
                '-f', 'image2pipe',
                '-vf', f'scale={DESIRED_WIDTH}:{DESIRED_HEIGHT}',
                '-'
            ]

            ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)
            while not self.stop_event.is_set():
                raw_frame = ffmpeg_process.stdout.read(DESIRED_WIDTH * DESIRED_HEIGHT * 3)
                if len(raw_frame) != DESIRED_WIDTH * DESIRED_HEIGHT * 3:
                    break

                frame = np.frombuffer(raw_frame, np.uint8).reshape((DESIRED_HEIGHT, DESIRED_WIDTH, 3))

                if not self.frame_queue.full():
                    self.frame_queue.put(frame, block=True)
                else:
                    time.sleep(0.01)

            ffmpeg_process.stdout.close()
            ffmpeg_process.stderr.close()
            ffmpeg_process.terminate()
            ffmpeg_process.wait()
        except Exception as e:
            logging.error(f"Ошибка чтения кадров: {e}")
        finally:
            self.stop_event.set()
            logging.info("Поток чтения видео завершен.")

    def display_frames(self):
        try:
            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    with self.lock:
                        stabilized_frame = cv2.warpPerspective(frame, np.eye(3), (frame.shape[1], frame.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
                    cv2.imshow('Video Stream', stabilized_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.stop_event.set()
                        break
                time.sleep(0.01)
        except Exception as e:
            logging.error(f"Ошибка отображения кадров: {e}")
        finally:
            cv2.destroyAllWindows()

    def start_reading_thread(self):
        self.thread_read = Thread(target=self.read_frames, daemon=True)
        self.thread_read.start()

    def start_display_thread(self):
        self.thread_display = Thread(target=self.display_frames, daemon=True)
        self.thread_display.start()

    def start_processing(self):
        try:
            start_time = time.perf_counter()
            max_duration = 300

            initial_x, initial_y = self.initial_coordinates
            logging.info(f"Передача начальных координат в setup: x_start={initial_x}, y_start={initial_y}")

            self.support_positioning.setup(self.frame_queue.get(), initial_x, initial_y)

            previous_frame_time = time.perf_counter()

            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    self.processed_frames += 1

                    frame_start_time = time.perf_counter()

                    if np.mean(frame) < 10:
                        logging.warning("Обнаружен серый кадр")
                    else:
                        if self.processed_frames % COORDINATE_INTERVAL == 0:
                            coordinate = self.support_positioning.calculate_coordinate(frame, initial_x, initial_y)
                            logging.info(f"Координаты на основании текущего кадра: {coordinate}")

                            socket_server.send_coordinates(coordinate[0], coordinate[1])

                    frame_end_time = time.perf_counter()
                    processing_time = frame_end_time - frame_start_time
                    logging.info(f"Время обработки кадра {self.processed_frames}: {processing_time:.2f} секунд")

                    previous_frame_time = frame_end_time

                if time.perf_counter() - start_time > max_duration:
                    logging.info("Достигнута максимальная длительность обработки")
                    break

                time.sleep(1.0 / FPS)
        except KeyboardInterrupt:
            logging.info("Программа завершена пользователем.")
        except Exception as e:
            logging.error(f"Ошибка обработки кадра: {e}")
        finally:
            self.stop_event.set()
            if self.thread_read:
                self.thread_read.join(timeout=1)

if __name__ == "__main__":
    stop_event = Event()
    lock = Lock()
    video_processor = VideoProcessor(VIDEO_PATH)
    video_processor.start_reading_thread()
    video_processor.start_display_thread()
    try:
        video_processor.start_processing()
    finally:
        stop_event.set()
        if video_processor.thread_read:
            video_processor.thread_read.join()
        if video_processor.thread_display:
            video_processor.thread_display.join()
        cv2.destroyAllWindows()
        logging.info("Программа завершена.")
