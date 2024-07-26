"""support_positioning.py"""

import logging
import os
import cv2
import time
# import csv
from collections import deque
from math import tan, radians, cos, sin
from pymavlink import mavutil
# import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('Agg')  # Использовать non-interactive backend

# Настройка логирования
logging.basicConfig(filename='/home/adam/Documents/HermesLocate/logs/flight_log.log', level=logging.INFO, format='%(message)s')

def rotate_point(x, y, angle_degrees):
    """Вращает точку на плоскости на заданный угол."""
    angle_radians = radians(angle_degrees)
    x_new = x * cos(angle_radians) - y * sin(angle_radians)
    y_new = x * sin(angle_radians) + y * cos(angle_radians)
    return x_new, y_new

class SupportPositioning:
    def __init__(self, master, threshold_coefficient=0.1):
        """Инициализация класса SupportPositioning."""
        self.threshold_distance = None
        self.reference_frame = None
        self.reference_keypoints = None
        self.reference_descriptors = None
        self.entries = deque()
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.threshold_coefficient = threshold_coefficient
        self.master = master
        # self.csv_filename = '/home/adam/Documents/droneTest/logs/trajectory.csv'
        self.initialized = False

    # def initialize_csv(self):
    #     """Инициализирует CSV файл."""
    #     with open(self.csv_filename, mode='w', newline='') as file:
    #         writer = csv.writer(file)
    #         writer.writerow(['X Coordinate', 'Y Coordinate'])
    #     logging.info(f"CSV файл инициализирован: {self.csv_filename}")

    def setup(self, image, x_start, y_start):
        """Устанавливает начальное состояние и вычисляет пороговое расстояние."""
        self.reference_descriptors = None
        self.reference_keypoints = None
        if not self.initialized:
            logging.debug(f"Получены начальные координаты: x_start={x_start}, y_start={y_start}")
            print(f"DEBUG: Получены начальные координаты: x_start={x_start}, y_start={y_start}")
            self.x_coordinate = x_start
            self.y_coordinate = y_start
            self.reference_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.reference_keypoints, self.reference_descriptors = self.orb.detectAndCompute(self.reference_frame, None)
            self.calculate_threshold_distance(image)
            self.initialized = True
            logging.info(f"Инициализация завершена с координатами: x_start={x_start}, y_start={y_start}, threshold_distance={self.threshold_distance}")

    def calculate_threshold_distance(self, image):
        """Вычисляет пороговое расстояние на основе размера изображения."""
        height, width = image.shape[:2]
        self.threshold_distance = self.threshold_coefficient * min(height, width)
        logging.info(f"Пороговое расстояние рассчитано: threshold_distance={self.threshold_distance}")

    def get_azimuth(self):
        """Получает текущий азимут из сообщений MAVLink."""
        while True:
            msg = self.master.recv_match(type='VFR_HUD', blocking=True)
            if msg:
                logging.info(f"Азимут: {msg.heading}")
                return msg.heading

    def get_altitude(self):
        """Получает текущую высоту из сообщений MAVLink."""
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                logging.info(f"Высота: {msg.relative_alt / 1000.0}")
                return msg.relative_alt / 1000.0

    def calculate_coordinate(self, image, x_start, y_start):
        """Вычисляет координаты на основе текущего кадра."""
        if self.reference_keypoints is None:
            self.setup(image, x_start, y_start)
            return self.x_coordinate, self.y_coordinate

        current_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_keypoints, current_descriptors = self.orb.detectAndCompute(current_frame, None)
        matches = self.bf.match(self.reference_descriptors, current_descriptors)
        good_matches = [m for m in matches if m.distance < self.threshold_distance]

        if len(good_matches) == 0:
            logging.warning("Нет хороших совпадений ключевых точек.")
            return self.x_coordinate, self.y_coordinate

        # Вычисление среднего смещения по X и Y на основе хороших совпадений
        displacement_x = sum(current_keypoints[m.trainIdx].pt[0] - self.reference_keypoints[m.queryIdx].pt[0] for m in good_matches) / len(good_matches)
        displacement_y = sum(current_keypoints[m.trainIdx].pt[1] - self.reference_keypoints[m.queryIdx].pt[1] for m in good_matches) / len(good_matches)
        logging.info(f"Смещение по X: {displacement_x}, Смещение по Y: {displacement_y}")

        azim = self.get_azimuth()
        alt = self.get_altitude()
        fov = 42

        # Вычисление координат на основе азимута, высоты, поля зрения и смещения пикселей
        coordinate = self.calculate_coordinate_xy(azim, 122, fov, displacement_x, displacement_y, image.shape[1], image.shape[0])
        self.entries.append((coordinate[0], coordinate[1]))
        self.update_reference_frame(current_frame, current_keypoints, current_descriptors)

        logging.info(f"Координаты по текущему кадру: (x={coordinate[0]}, y={coordinate[1]})")

        # Сохранение новых координат в CSV файл
        #self.save_trajectory_to_csv()
        #logging.info(f"Координаты сохранены в CSV: (x={coordinate[0]}, y={coordinate[1]})")

        return coordinate[0], coordinate[1]

    def calculate_coordinate_xy(self, azim, alt, fov, dx_pixels, dy_pixels, window_width, window_height):
        """Вычисляет координаты x и y на основе углов и смещений пикселей."""
        tan_half_fov = abs(alt * tan(radians(fov)))
        dx_metres = (2 * tan_half_fov) / window_width
        dy_metres = (2 * tan_half_fov) / window_height
        x_distance = dx_pixels * dx_metres
        y_distance = dy_pixels * dy_metres

        # Поворот координат на текущий азимут
        rotate = rotate_point(x_distance, y_distance, azim)
        self.x_coordinate += rotate[1]
        self.y_coordinate -= rotate[0]
        logging.info(f"Обновленные координаты: x_coordinate={self.x_coordinate}, y_coordinate={self.y_coordinate}")
        return self.x_coordinate, self.y_coordinate

    def update_reference_frame(self, current_frame, current_keypoints, current_descriptors):
        """Обновляет опорный кадр для дальнейших вычислений."""
        self.reference_frame = current_frame
        self.reference_keypoints = current_keypoints
        self.reference_descriptors = current_descriptors
        logging.info("Опорный кадр обновлен.")

    def get_coordinates(self):
        """Возвращает текущие координаты."""
        return self.x_coordinate, self.y_coordinate

    # def visualize_trajectory(self):
    #     """Визуализирует траекторию на графике."""
    #     if self.entries:
    #         x_coords, y_coords = zip(*self.entries)
    #         plt.plot(x_coords, y_coords, marker='o')
    #         plt.gca().invert_yaxis()
    #         plt.title('Trajectory')
    #         plt.xlabel('X Coordinate')
    #         plt.ylabel('Y Coordinate')
    #         plt.show()

    # def save_trajectory_to_image(self, base_filename='/home/adam/Documents/droneTest/logs/trajectory', background_image_path=None):
    #     """Сохраняет траекторию в виде изображения."""
    #     timestamp = time.strftime("%Y%m%d-%H%M%S")
    #     filename = f"{base_filename}_{timestamp}.png"

    #     if self.entries:
    #         x_coords, y_coords = zip(*self.entries)
    #         fig, ax = plt.subplots()
    #         if background_image_path:
    #             img = plt.imread(background_image_path)
    #             ax.imshow(img, extent=[min(x_coords), max(x_coords), max(y_coords), min(y_coords)])

    #         ax.plot(x_coords, y_coords, marker='o')
    #         ax.invert_yaxis()
    #         ax.set_title('Trajectory')
    #         ax.set_xlabel('X Coordinate')
    #         ax.set_ylabel('Y Coordinate')
    #         plt.savefig(filename)
    #         plt.close()
    #         logging.info(f"Траектория сохранена в {filename}")

    # def save_trajectory_to_csv(self):
    #     """Сохраняет траекторию в CSV файл."""
    #     with open(self.csv_filename, mode='a', newline='') as file:
    #         writer = csv.writer(file)
    #         for entry in self.entries:
    #             writer.writerow(entry)
    #     logging.info(f"Траектория сохранена в CSV файл: {self.csv_filename}")

    # def close_csv(self):
    #     """Закрывает CSV файл."""
    #     logging.info("Закрытие CSV файла.")