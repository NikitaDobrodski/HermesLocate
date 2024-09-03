"""main_video.py"""

#!/usr/bin/python3

import cv2
import queue
import time
from threading import Thread, Event, Lock
from pymavlink import mavutil
import os
import tracemalloc
import signal
import sys
from socket_server import SocketServer
from support_positioning import SupportPositioning

os.environ["QT_QPA_PLATFORM"] = "xcb"

def signal_handler(sig, frame):
    print("Signal received, stopping...")
    stop_event.set()
    video_processor.cleanup_threads()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

VIDEO_PATH = "rtsp://adam:1984@192.168.144.26:8554/main.264"
DESIRED_WIDTH = 480
DESIRED_HEIGHT = 360
COORDINATE_INTERVAL = 5
FPS = 30

socket_server = SocketServer('192.168.144.5', 5008)

tracemalloc.start()

def start_server():
    socket_server.start()

server_thread = Thread(target=start_server, daemon=True)
server_thread.start()

stop_event = Event()
frame_queue = queue.Queue(maxsize=10)

def read_frames(video_path, frame_queue, stop_event):
    stream = cv2.VideoCapture(video_path)
    
    if not stream.isOpened():
        print("Error: Unable to open video stream.")
        stop_event.set()
        return

    try:
        while not stop_event.is_set():
            ret, frame = stream.read()
            if not ret:
                print("Error: Unable to read frame.")
                break

            frame = cv2.resize(frame, (DESIRED_WIDTH, DESIRED_HEIGHT))

            if frame_queue.full():
                # Remove oldest frame to make space for new frame
                frame_queue.get()

            frame_queue.put(frame)

    except Exception as e:
        print(f"Error reading frames: {e}")
    finally:
        stream.release()

def display_frames(frame_queue, stop_event):
    try:
        while not stop_event.is_set():
            if not frame_queue.empty():
                frame = frame_queue.get()
                cv2.imshow('Video Stream', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    break
            time.sleep(0.01)
    except Exception as e:
        print(f"Error displaying frames: {e}")
    finally:
        cv2.destroyAllWindows()

class VideoProcessor:
    def __init__(self, frame_queue):
        self.frame_queue = frame_queue
        self.stop_event = stop_event
        self.thread_processing = None
        self.master = self.initialize_mavlink()
        self.initial_coordinates = self.get_initial_coordinates()
        self.support_positioning = SupportPositioning(self.master)
        self.processed_frames = 0

    def initialize_mavlink(self):
        try:
            master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
            return master
        except Exception as e:
            print(f"Error initializing MAVLink: {e}")
            sys.exit(1)

    def get_initial_coordinates(self):
        while True:
            data = socket_server.receive_data()
            if data:
                try:
                    x_start, y_start = map(float, data.split(','))
                    return x_start, y_start
                except ValueError:
                    print("Error parsing coordinates.")
                    continue

    def start_processing_thread(self):
        self.thread_processing = Thread(target=self.start_processing, daemon=True)
        self.thread_processing.start()

    def start_processing(self):
        try:
            initial_x, initial_y = self.initial_coordinates
            self.support_positioning.setup(self.frame_queue.get(), initial_x, initial_y)

            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    self.processed_frames += 1

                    frame_start_time = time.perf_counter()

                    if self.processed_frames % COORDINATE_INTERVAL == 0:
                        coordinate = self.support_positioning.calculate_coordinate(frame, initial_x, initial_y)
                        socket_server.send_coordinates(*coordinate)

                    frame_end_time = time.perf_counter()
                    processing_time = frame_end_time - frame_start_time

                    time.sleep(1 / FPS)
        except Exception as e:
            print(f"Error processing frames: {e}")
        finally:
            self.stop_event.set()

    def cleanup_threads(self):
        if self.thread_processing:
            self.thread_processing.join(timeout=1)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    lock = Lock()

    # Start reading frames from RTSP stream in a separate thread
    reading_thread = Thread(target=read_frames, args=(VIDEO_PATH, frame_queue, stop_event), daemon=True)
    reading_thread.start()

    # Start displaying frames in a separate thread
    display_thread = Thread(target=display_frames, args=(frame_queue, stop_event), daemon=True)
    display_thread.start()

    video_processor = VideoProcessor(frame_queue)
    video_processor.start_processing_thread()

    try:
        while not stop_event.is_set():
            time.sleep(1)
    finally:
        stop_event.set()
        video_processor.cleanup_threads()
        reading_thread.join(timeout=1)
        display_thread.join(timeout=1)

    snapshot = tracemalloc.take_snapshot()
    top_stats = snapshot.statistics('lineno')

    for stat in top_stats[:10]:
        print(stat)
