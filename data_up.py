" Тестовый код для подмены данных gps"

"data_up.py"

import time
from pymavlink import mavutil
import random

def generate_gps_data():
    """
    Возвращает фиксированные данные GPS, включая количество спутников.
    Эти данные могут быть изменены для моделирования различных условий GPS.
    """
    # Фиксированные данные GPS
    latitude = 56.17 + random.uniform(-0.5, 0.5)
    longitude = 73.28 + random.uniform(-0.5, 0.5)
    altitude = 100.0 + random.uniform(100, 500)
    satellites = random.randint(50, 60)
    return latitude, longitude, altitude, satellites

def main():
    # Создаем MAVLink соединение через указанный порт и с заданной скоростью передачи данных (baudrate)
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    
    # Ожидание сигнала от сердца (heartbeat) от подключенного устройства.
    # Этот сигнал показывает, что устройство активно и готово к работе.
    master.wait_heartbeat()

    try:
        while True:
            # Генерация GPS данных с использованием фиксированных значений
            latitude, longitude, altitude, satellites = generate_gps_data()

            # Преобразование широты и долготы в формат, ожидаемый MAVLink
            # Формат - широта и долгота умноженные на 10^7 (целочисленное значение)
            lat = int(latitude * 1e7)
            lon = int(longitude * 1e7)

            # Высота уже представлена в метрах как float, преобразование не требуется
            alt = float(altitude)

            # Вывод сгенерированных данных для отладки
            print(f"lat={lat}, lon={lon}, alt={alt}, satellites={satellites}")

            # Отправка GPS данных через MAVLink
            master.mav.gps_input_send(
                int(time.time() * 1e6),  # Время в микросекундах с момента Unix Epoch
                0,  # ID устройства (0 - основное устройство)
                0,  # Флаги GPS (не используются)
                0,  # Тип GPS (0 - не указан)
                0,  # ID спутниковой системы (0 - не указан)
                3,  # Режим фиксации GPS (3 = 3D фиксация, т.е. получены координаты и высота)
                lat,  # Широта в формате int32
                lon,  # Долгота в формате int32
                alt,  # Высота над уровнем моря в метрах (float)
                float(0.7),  # Горизонтальное рассогласование DOP (HDOP)
                float(1.0),  # Вертикальное рассогласование DOP (VDOP)
                0,  # Скорость по земле (не используется)
                0,  # Скорость по вертикали (не используется)
                0,  # Курс (не используется)
                float(0.1),  # Вертикальная ошибка (не используется)
                float(0.5),  # Горизонтальная ошибка (не используется)
                float(0.5),  # Скоростная ошибка (не используется)
                satellites,  # Количество спутников
                0  # Yaw (не используется)
            )
            
            
    
    except KeyboardInterrupt:
        # Обработка завершения программы пользователем (Ctrl+C)
        print("Прервано пользователем.")
    except Exception as e:
        # Обработка других исключений с выводом сообщения об ошибке
        print(f"Произошла ошибка: {e}")
    finally:
        # Закрытие соединения MAVLink при завершении работы
        master.close()

# Проверка, что скрипт запущен как основная программа
if __name__ == '__main__':
    main()
