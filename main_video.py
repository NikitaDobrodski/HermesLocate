"""main_video.py"""

#!/usr/bin/python3

import cv2
import queue
import time
import numpy as np
import logging
from threading import Thread, Event
from pymavlink import mavutil
from support_positioning import SupportPositioning
import signal
import sys
import subprocess
import os
from socket_server import SocketServer

def signal_handler(sig, frame):
    """Обработчик сигналов для корректного завершения программы."""
    logging.info('Программа прервана пользователем!')
    sys.exit(0)

# Установка обработчиков сигналов для корректного завершения программы
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Директория для логов
log_dir = '/home/adam/Documents/HermesLocate/logs'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# Настройка логирования
logging.basicConfig(filename=os.path.join(log_dir, 'flight_log.log'), level=logging.INFO, format='%(asctime)s - %(message)s')

# Параметры видео
VIDEO_PATH = "/home/adam/Documents/HermesLocate/movies/output.mp4" # rtsp://adam:1984@192.168.144.25:8554/main.264
DESIRED_WIDTH = 480
DESIRED_HEIGHT = 360
COORDINATE_INTERVAL = 5
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
        """Инициализация объекта VideoProcessor."""
        self.video_path = video_path
        self.stop_event = Event()
        self.frame_queue = queue.Queue(maxsize=10)
        self.thread_read = None
        self.thread_display = None
        self.master = self.initialize_mavlink()
        self.initial_coordinates = self.get_initial_coordinates()
        self.support_positioning = SupportPositioning(self.master)
        self.processed_frames = 0

    def initialize_mavlink(self):
        """Инициализация соединения MAVLink."""
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
        """Получение начальных координат от клиента через UDP."""
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
        """Чтение кадров из видеопотока."""
        try:
            # Команда ffmpeg для чтения видеопотока
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

            # Запуск процесса ffmpeg
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

            # Закрытие процессов ffmpeg
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
        """Отображение кадров видеопотока."""
        try:
            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    cv2.imshow('Video Stream', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.stop_event.set()
                        break
                time.sleep(0.01)
        except Exception as e:
            logging.error(f"Ошибка отображения кадров: {e}")
        finally:
            cv2.destroyAllWindows()

    def start_reading_thread(self):
        """Запуск потока для чтения кадров."""
        self.thread_read = Thread(target=self.read_frames, daemon=True)
        self.thread_read.start()

    def start_display_thread(self):
        """Запуск потока для отображения кадров."""
        self.thread_display = Thread(target=self.display_frames, daemon=True)
        self.thread_display.start()

    def start_processing(self):
        """Запуск обработки кадров."""
        try:
            start_time = time.time()
            max_duration = 300

            initial_x, initial_y = self.initial_coordinates
            logging.info(f"Передача начальных координат в setup: x_start={initial_x}, y_start={initial_y}")

            # Настройка поддержки позиционирования
            self.support_positioning.setup(self.frame_queue.get(), initial_x, initial_y)

            previous_frame_time = time.time()

            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    self.processed_frames += 1

                    if np.mean(frame) < 10:
                        logging.warning("Обнаружен серый кадр")
                    else:
                        if self.processed_frames % COORDINATE_INTERVAL == 0:
                            coordinate = self.support_positioning.calculate_coordinate(frame, initial_x, initial_y)
                            logging.info(f"Координаты на основании текущего кадра: {coordinate}")

                            # Отправка абсолютных координат через SocketServer
                            socket_server.send_coordinates(coordinate[0], coordinate[1])

                    current_frame_time = time.time()
                    processing_time = current_frame_time - previous_frame_time
                    logging.info(f"Время обработки кадра {self.processed_frames}: {processing_time:.2f} секунд")
                    previous_frame_time = current_frame_time

                if time.time() - start_time > max_duration:
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
            # logging.info("Программа завершена.")
            # self.support_positioning.visualize_trajectory()
            # logging.info("Сохранение изображения траектории")
            # self.support_positioning.save_trajectory_to_image()
            # logging.info("Сохранение CSV траектории")
            # self.support_positioning.save_trajectory_to_csv()
            # socket_server.stop()

if __name__ == "__main__":
    video_processor = VideoProcessor(VIDEO_PATH)
    #video_processor.support_positioning.initialize_csv()
    video_processor.start_reading_thread()
    video_processor.start_display_thread()
    #try:
    video_processor.start_processing()
    # finally:
        #video_processor.support_positioning.close_csv()