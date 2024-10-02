"""support_positioning.py"""

import cv2
from collections import deque
from math import tan, radians, cos, sin
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
import numpy as np  # Медиану будем считать через numpy

def rotate_point(x, y, angle_degrees):
    """Вращает точку на заданный угол в градусах."""
    angle_radians = radians(angle_degrees)
    x_new = x * cos(angle_radians) - y * sin(angle_radians)
    y_new = x * sin(angle_radians) + y * cos(angle_radians)
    return x_new, y_new

class SupportPositioning:
    def __init__(self, master, threshold_coefficient=0.1):
        self.threshold_distance = None
        self.reference_frame = None
        self.reference_keypoints = None
        self.reference_descriptors = None
        self.entries = deque(maxlen=250)  # Фиксированный размер буфера для координат
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.threshold_coefficient = threshold_coefficient
        self.master = master
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.lock = Lock()
        self.initialized = False

        # Храним последние 15 смещений по X и Y для сглаживания
        self.displacements_x = deque(maxlen=15)
        self.displacements_y = deque(maxlen=15)

    def setup(self, image, x_start, y_start):
        """Настраивает эталонные ключевые точки и дескрипторы."""
        if not self.initialized:
            self.x_coordinate = x_start
            self.y_coordinate = y_start

            self.reference_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.reference_keypoints, self.reference_descriptors = self.orb.detectAndCompute(self.reference_frame, None)
            if self.reference_keypoints is None or self.reference_descriptors is None:
                return
            self.calculate_threshold_distance(image)
            self.initialized = True

    def calculate_threshold_distance(self, image):
        """Рассчитывает пороговое расстояние для совпадений ключевых точек."""
        height, width = image.shape[:2]
        self.threshold_distance = self.threshold_coefficient * min(height, width)

    def get_azimuth(self):
        """Получает азимут из MAVLink."""
        while True:
            try:
                msg = self.master.recv_match(type='VFR_HUD', blocking=True)
                if msg:
                    return msg.heading
            except Exception:
                continue

    def get_altitude(self):
        """Получает высоту из MAVLink."""
        while True:
            try:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    return msg.relative_alt / 1000.0
            except Exception:
                continue

    @staticmethod
    def preprocess_image(image):
        """Предобрабатывает изображение для улучшения качества ключевых точек."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        equalized = cv2.equalizeHist(gray)
        blurred = cv2.GaussianBlur(equalized, (5, 5), 0)
        thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        return thresholded

    def calculate_coordinate(self, image, x_start, y_start):
        """Рассчитывает координаты на основе текущего кадра."""
        with self.lock:
            try:
                if not self.initialized:
                    self.setup(image, x_start, y_start)

                if self.reference_keypoints is None or self.reference_descriptors is None:
                    return self.x_coordinate, self.y_coordinate

                current_frame_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                current_keypoints, current_descriptors = self.orb.detectAndCompute(current_frame_gray, None)

                # Проверка дескрипторов
                if current_descriptors is None or self.reference_descriptors is None:
                    return self.x_coordinate, self.y_coordinate

                matches = self.bf.match(self.reference_descriptors, current_descriptors)
                good_matches = [m for m in matches if m.distance < self.threshold_distance]

                if not good_matches:
                    return self.x_coordinate, self.y_coordinate

                # Используйте numpy для получения смещений
                displacements = np.array([
                    (
                        current_keypoints[m.trainIdx].pt[0] - self.reference_keypoints[m.queryIdx].pt[0],
                        current_keypoints[m.trainIdx].pt[1] - self.reference_keypoints[m.queryIdx].pt[1]
                    ) for m in good_matches
                ])

                median_displacement = np.median(displacements, axis=0)

                azim = self.get_azimuth()
                alt = self.get_altitude()
                fov = 42  # Поле зрения камеры в градусах

                coordinate = self.calculate_coordinate_xy(
                    azim, 100, fov,
                    median_displacement[0], median_displacement[1],
                    image.shape[1], image.shape[0]
                )

                self.entries.append((coordinate[0], coordinate[1]))

                # Обновляем ссылочный кадр в фоновом потоке, используя временной интервал
                if self.should_update_reference():  # Условие для обновления
                    self.executor.submit(self.update_reference_frame, current_frame_gray, current_keypoints, current_descriptors)

                return coordinate[0], coordinate[1]

            except Exception as e:
                print(f"Error in calculate_coordinate: {e}")
                return self.x_coordinate, self.y_coordinate

    def should_update_reference(self):
        """Определяет, нужно ли обновлять ссылочный кадр."""
        # Логика для определения, когда следует обновить кадр
        return len(self.entries) % 10 == 0  # Например, обновлять каждые 10 записей

    def calculate_coordinate_xy(self, azim, alt, fov, dx_pixels, dy_pixels, window_width, window_height):
        """Вычисляет координаты на основе смещения в пикселях и параметров камеры."""
        try:
            # Вычисляем полуширину поля зрения в градусах
            tan_half_fov = abs(tan(radians(fov / 2)))
            
            # Примерный коэффициент для перевода пикселей в метры
            meters_per_pixel_x = (tan_half_fov * alt) / (window_width / 2)
            meters_per_pixel_y = (tan_half_fov * alt) / (window_height / 2)

            # 1 градус широты ≈ 111.32 км
            lat_per_meter = 1 / 111320.0

            # 1 градус долготы ≈ 111.32 км * cos(широта)
            lon_per_meter = 1 / (111320.0 * cos(radians(self.x_coordinate)))

            # Преобразуем смещения из пикселей в градусы
            delta_lat = dy_pixels * meters_per_pixel_y * lat_per_meter
            delta_lon = dx_pixels * meters_per_pixel_x * lon_per_meter

            # Применяем поворот по азимуту
            rotated = rotate_point(delta_lon, delta_lat, azim)

            # Обновляем координаты
            self.x_coordinate += rotated[1]  # Широта
            self.y_coordinate += rotated[0]  # Долгота

            return self.x_coordinate, self.y_coordinate
        except Exception:
            return self.x_coordinate, self.y_coordinate

    def update_reference_frame(self, current_frame, current_keypoints, current_descriptors):
        """Обновляет эталонный кадр и ключевые точки."""
        with self.lock:
            try:
                self.reference_frame = current_frame
                self.reference_keypoints = current_keypoints
                self.reference_descriptors = current_descriptors
            except Exception:
                pass

    def get_coordinates(self):
        """Возвращает текущие координаты."""
        with self.lock:
            return self.x_coordinate, self.y_coordinate

    def process_frame(self, image, x_start, y_start):
        """Обрабатывает текущий кадр и рассчитывает координаты."""
        try:
            self.setup(image, x_start, y_start)
            coordinate = self.calculate_coordinate(image, x_start, y_start)
            return coordinate
        except Exception:
            return None