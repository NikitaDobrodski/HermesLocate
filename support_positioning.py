"""support_positioning.py"""

import logging
import os
import cv2
import time
from collections import deque
from math import tan, radians, cos, sin
from concurrent.futures import ThreadPoolExecutor
from pymavlink import mavutil

# Настройка логирования
logging.basicConfig(filename='/home/adam/Documents/HermesLocate/logs/flight_log.log', level=logging.INFO, format='%(message)s')

def rotate_point(x, y, angle_degrees):
    angle_radians = radians(angle_degrees)  # Преобразование угла в радианы
    x_new = x * cos(angle_radians) - y * sin(angle_radians)  # Новая координата x после поворота
    y_new = x * sin(angle_radians) + y * cos(angle_radians)  # Новая координата y после поворота
    return x_new, y_new

class SupportPositioning:
    def __init__(self, master, threshold_coefficient=0.1):
        self.threshold_distance = None  # Пороговое расстояние для фильтрации совпадений
        self.reference_frame = None  # Опорный кадр для сопоставления
        self.reference_keypoints = None  # Ключевые точки опорного кадра
        self.reference_descriptors = None  # Описание ключевых точек опорного кадра
        self.entries = deque()  # Очередь для хранения вычисленных координат
        self.orb = cv2.ORB_create()  # Создание объекта ORB для детектирования и описания ключевых точек
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # Создание объекта для сопоставления дескрипторов
        self.threshold_coefficient = threshold_coefficient  # Коэффициент порога для фильтрации совпадений
        self.master = master  # Соединение MAVLink
        self.executor = ThreadPoolExecutor(max_workers=2)  # Пул потоков для асинхронного обновления опорного кадра
        self.initialized = False  # Флаг, указывающий, инициализирован ли объект

    def setup(self, image, x_start, y_start):
        self.reference_descriptors = None
        self.reference_keypoints = None
        if not self.initialized:
            logging.debug(f"Получены начальные координаты: x_start={x_start}, y_start={y_start}")
            self.x_coordinate = x_start
            self.y_coordinate = y_start
            # Преобразование опорного кадра в градации серого
            self.reference_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Выявление ключевых точек и дескрипторов опорного кадра
            self.reference_keypoints, self.reference_descriptors = self.orb.detectAndCompute(self.reference_frame, None)
            # Расчет порогового расстояния
            self.calculate_threshold_distance(image)
            self.initialized = True
            logging.info(f"Инициализация завершена с координатами: x_start={x_start}, y_start={y_start}, threshold_distance={self.threshold_distance}")

    def calculate_threshold_distance(self, image):
        height, width = image.shape[:2]
        self.threshold_distance = self.threshold_coefficient * min(height, width)  # Пороговое расстояние
        logging.info(f"Пороговое расстояние рассчитано: threshold_distance={self.threshold_distance}")

    def get_azimuth(self):
        while True:
            msg = self.master.recv_match(type='VFR_HUD', blocking=True)  # Получение сообщения с азимутом
            if msg:
                logging.info(f"Азимут: {msg.heading}")
                return msg.heading

    def get_altitude(self):
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  # Получение сообщения с высотой
            if msg:
                logging.info(f"Высота: {msg.relative_alt / 1000.0}")
                return msg.relative_alt / 1000.0  # Преобразование высоты из миллиметров в метры

    def calculate_coordinate(self, image, x_start, y_start):
        if self.reference_keypoints is None:
            self.setup(image, x_start, y_start)  # Инициализация при отсутствии опорных данных
            return self.x_coordinate, self.y_coordinate

        # Преобразование текущего кадра в градации серого
        current_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Выявление ключевых точек и дескрипторов текущего кадра
        current_keypoints, current_descriptors = self.orb.detectAndCompute(current_frame, None)
        # Сопоставление дескрипторов
        matches = self.bf.match(self.reference_descriptors, current_descriptors)
        good_matches = [m for m in matches if m.distance < self.threshold_distance]  # Фильтрация хороших совпадений

        if len(good_matches) == 0:
            logging.warning("Нет хороших совпадений ключевых точек.")
            return self.x_coordinate, self.y_coordinate

        # Расчет смещений по x и y
        displacement_x = sum(current_keypoints[m.trainIdx].pt[0] - self.reference_keypoints[m.queryIdx].pt[0] for m in good_matches) / len(good_matches)
        displacement_y = sum(current_keypoints[m.trainIdx].pt[1] - self.reference_keypoints[m.queryIdx].pt[1] for m in good_matches) / len(good_matches)
        logging.info(f"Смещение по X: {displacement_x}, Смещение по Y: {displacement_y}")

        azim = self.get_azimuth()  # Получение текущего азимута
        alt = self.get_altitude()  # Получение текущей высоты
        fov = 42  # Поле зрения камеры в градусах

        # Вычисление координат на основе смещений и параметров камеры
        coordinate = self.calculate_coordinate_xy(azim, 20, fov, displacement_x, displacement_y, image.shape[1], image.shape[0])
        self.entries.append((coordinate[0], coordinate[1]))  # Добавление координат в очередь
        # Асинхронное обновление опорного кадра
        self.executor.submit(self.update_reference_frame, current_frame, current_keypoints, current_descriptors)

        logging.info(f"Координаты по текущему кадру: (x={coordinate[0]}, y={coordinate[1]})")

        return coordinate[0], coordinate[1]

    def calculate_coordinate_xy(self, azim, alt, fov, dx_pixels, dy_pixels, window_width, window_height):
        tan_half_fov = abs(alt * tan(radians(fov)))  # Полуфов в метрах
        dx_metres = (2 * tan_half_fov) / window_width  # Перевод смещения по x в метры
        dy_metres = (2 * tan_half_fov) / window_height  # Перевод смещения по y в метры
        x_distance = dx_pixels * dx_metres  # Расчет смещения по x в метрах
        y_distance = dy_pixels * dy_metres  # Расчет смещения по y в метрах

        # Поворот координат с учетом азимута
        rotate = rotate_point(x_distance, y_distance, azim)
        self.x_coordinate += rotate[1]
        self.y_coordinate -= rotate[0]
        logging.info(f"Обновленные координаты: x_coordinate={self.x_coordinate}, y_coordinate={self.y_coordinate}")
        return self.x_coordinate, self.y_coordinate

    def update_reference_frame(self, current_frame, current_keypoints, current_descriptors):
        self.reference_frame = current_frame
        self.reference_keypoints = current_keypoints
        self.reference_descriptors = current_descriptors

    def get_coordinates(self):
        return self.x_coordinate, self.y_coordinate