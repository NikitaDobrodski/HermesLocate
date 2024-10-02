# main_video.py

#!/usr/bin/python3

import cv2
import time
import threading
from multiprocessing import Process, Queue, Event
from pymavlink import mavutil
import os
import signal
import sys
from socket_server import SocketServer
from support_positioning import SupportPositioning

# Устанавливаем платформу для Qt
os.environ["QT_QPA_PLATFORM"] = "xcb"

def signal_handler(sig, frame):
    print("Signal received, stopping...")
    stop_event.set()
    video_processor.cleanup_processes()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

VIDEO_PATH = "rtsp://adam:1984@192.168.144.26:8554/main.264"
DESIRED_WIDTH = 640
DESIRED_HEIGHT = 480
COORDINATE_INTERVAL = 3
FPS = 30  # Уменьшение FPS для уменьшения нагрузки на процессор

socket_server = SocketServer('192.168.144.5', 5008)

# Функция для запуска сервера
def start_server():
    socket_server.start()

stop_event = Event()
frame_queue = Queue(maxsize=10)  # Уменьшен размер очереди

class VideoReader:
    def __init__(self, video_path, frame_queue, stop_event, stream):
        self.video_path = video_path
        self.frame_queue = frame_queue
        self.stop_event = stop_event
        self.stream_aw = stream
        self.last_frame = None
        self.fps_control_interval = 1 / FPS
        self.last_frame_time = time.perf_counter()

class GpsController:
    def __init__(self, master):
        self.master = master

    def send_gps_to_controller(self, lat, lon, satellites=55):
        try:
            lat_mavlink = int(lat * 1e7)
            lon_mavlink = int(lon * 1e7)

            self.master.mav.gps_input_send(
                int(time.time() * 1e6),
                0,
                0,
                0,
                0,
                3,
                lat_mavlink,
                lon_mavlink,
                0,
                0.7,
                1.0,
                0,
                0,
                0,
                0.1,
                0.5,
                0.5,
                satellites,
                0
            )
            print(f"GPS данные отправлены: lat={lat}, lon={lon}")
        except Exception as e:
            print(f"Ошибка при отправке данных на MAVLink: {e}")

class VideoProcessor:
    def __init__(self, frame_queue, socket_server):
        self.frame_queue = frame_queue
        self.socket_server = socket_server
        self.stop_event = stop_event
        self.processed_frames = 0
        self.master = self.initialize_mavlink()
        self.gps_controller = GpsController(self.master)  # Use GpsController
        self.initial_coordinates = self.get_initial_coordinates()
        self.support_positioning = SupportPositioning(self.master)

    def initialize_mavlink(self):
        try:
            master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
            master.wait_heartbeat()
            print("Соединение с MAVLink установлено.")
            return master
        except Exception as e:
            print(f"Ошибка при подключении MAVLink: {e}")
            sys.exit(1)

    def get_initial_coordinates(self):
        while True:
            data, client_address = socket_server.receive_data()
            if data:
                try:
                    x_start, y_start = map(float, data.split(','))
                    return x_start, y_start
                except ValueError:
                    print("Ошибка при парсинге координат.")
                    continue

    def start_processing(self):
        try:
            initial_x, initial_y = self.initial_coordinates
            self.support_positioning.setup(self.frame_queue.get(), initial_x, initial_y)

            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    self.processed_frames += 1

                    if self.processed_frames % COORDINATE_INTERVAL == 0:
                        coordinate = self.support_positioning.calculate_coordinate(frame, initial_x, initial_y)
                        latitude, longitude = coordinate

                        # Use GpsController to send the GPS data
                        self.gps_controller.send_gps_to_controller(latitude, longitude)

                    time.sleep(0.3)
        except Exception as e:
            print(f"Ошибка в процессе обработки видео: {e}")

    def cleanup_processes(self):
        pass

# Главная часть программы
if __name__ == "__main__":
    frame_queue = Queue()
    stream = cv2.VideoCapture(VIDEO_PATH)  # Подключение к камере, можно указать URL камеры

    # Запускаем сервер в отдельном процессе
    server_process = Process(target=start_server)
    server_process.start()

    try:
        while True:
            # Чтение кадра из потока
            ret, frame = stream.read()
            if not ret:
                print("Failed to grab frame.")
                break

            # Изменение размера кадра
            height, width, _ = frame.shape
            frame = cv2.resize(frame, (width // 2, height // 2))

            # Показ видео
            cv2.imshow("Video1", frame)

            # Добавление кадра в очередь
            frame_queue.put(frame)

            # Выход по нажатию клавиши 'x'
            if cv2.waitKey(1) & 0xFF == ord('x'):
                break
    except Exception as e:
        print("ERROR:", e)
    finally:
        stream.release()
        cv2.destroyAllWindows()
        
    # Запускаем процессор видео в отдельном процессе
    video_processor = VideoProcessor(frame_queue, server_process)
    video_processor_process = Process(target=video_processor.start_processing)
    video_processor_process.start()

    # Ожидание завершения всех процессов
    video_processor_process.join()
    server_process.join()
