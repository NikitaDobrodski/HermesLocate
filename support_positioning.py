"""support_positioning.py"""

import cv2
from collections import deque
from math import tan, radians, cos, sin
from concurrent.futures import ThreadPoolExecutor
from threading import Lock

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
        self.entries = deque(maxlen=100)  # Фиксированный размер буфера для координат
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.threshold_coefficient = threshold_coefficient
        self.master = master
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.lock = Lock()
        self.initialized = False

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
        """Расчитывает пороговое расстояние для совпадений ключевых точек."""
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
                if self.reference_keypoints is None or self.reference_descriptors is None:
                    self.setup(image, x_start, y_start)
                    if self.reference_keypoints is None or self.reference_descriptors is None:
                        return self.x_coordinate, self.y_coordinate

                current_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                current_keypoints, current_descriptors = self.orb.detectAndCompute(current_frame, None)

                if self.reference_descriptors is None or current_descriptors is None:
                    return self.x_coordinate, self.y_coordinate

                if self.reference_descriptors.shape[1] != current_descriptors.shape[1]:
                    return self.x_coordinate, self.y_coordinate

                matches = self.bf.match(self.reference_descriptors, current_descriptors)
                good_matches = [m for m in matches if m.distance < self.threshold_distance]

                if len(good_matches) == 0:
                    return self.x_coordinate, self.y_coordinate

                displacement_x = sum(current_keypoints[m.trainIdx].pt[0] - self.reference_keypoints[m.queryIdx].pt[0] for m in good_matches) / len(good_matches)
                displacement_y = sum(current_keypoints[m.trainIdx].pt[1] - self.reference_keypoints[m.queryIdx].pt[1] for m in good_matches) / len(good_matches)

                azim = self.get_azimuth()
                alt = self.get_altitude()
                fov = 42  # Поле зрения камеры в градусах

                coordinate = self.calculate_coordinate_xy(azim, 100, fov, displacement_x, displacement_y, image.shape[1], image.shape[0])
                self.entries.append((coordinate[0], coordinate[1]))
                self.executor.submit(self.update_reference_frame, current_frame, current_keypoints, current_descriptors)

                return coordinate[0], coordinate[1]

            except Exception:
                return self.x_coordinate, self.y_coordinate

    def calculate_coordinate_xy(self, azim, alt, fov, dx_pixels, dy_pixels, window_width, window_height):
        """Вычисляет координаты на основе смещения в пикселях и параметров камеры."""
        try:
            tan_half_fov = abs(alt * tan(radians(fov)))
            dx_metres = (2 * tan_half_fov) / window_width
            dy_metres = (2 * tan_half_fov) / window_height
            x_distance = dx_pixels * dx_metres
            y_distance = dy_pixels * dx_metres

            rotate = rotate_point(x_distance, y_distance, azim)
            self.x_coordinate += rotate[1]
            self.y_coordinate -= rotate[0]
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

