# HermesLocate

HermesLocate - это проект для стабилизации видео и определения координат с использованием OpenCV и MAVLink. Проект включает несколько модулей, таких как стабилизация видео, вычисление координат и сервер для обмена данными через сокеты.

## Содержание

- [Введение](#введение)
- [Структура проекта](#структура-проекта)
- [Требования](#требования)
- [Установка](#установка)
- [Конфигурация](#конфигурация)
- [Использование](#использование)
  - [Запуск сервера](#запуск-сервера)
  - [Запуск основного процесса](#запуск-основного-процесса)
- [Компоненты](#компоненты)
  - [main_video.py](#main_videopy)
  - [support_positioning.py](#support_positioningpy)
  - [socket_server.py](#socket_serverpy)
- [Логи](#логи)

## Введение

HermesLocate предназначен для стабилизации видео и определения координат объектов в видео в режиме реального времени. Этот проект использует библиотеку OpenCV для обработки изображений и pymavlink для взаимодействия с MAVLink-сообщениями.

## Структура проекта

- `main_video.py` - Главный файл, который запускает весь процесс обработки видео.
- `support_positioning.py` - Модуль для вычисления координат на основе видео.
- `socket_server.py` - Модуль, отвечающий за сервер сокетов для обмена данными.
- `logs/` - Директория для логов.

## Требования

- Python 3.x
- OpenCV
- pymavlink
- socket

## Установка

1. Клонируйте репозиторий:

    ```bash
    git clone https://github.com/yourusername/HermesLocate.git
    cd HermesLocate
    ```

2. Установите необходимые зависимости:

    ```bash
    pip install -r requirements.txt
    ```

    **Примечание:** Убедитесь, что у вас установлены OpenCV и pymavlink. Если они не включены в requirements.txt, установите их отдельно:

    ```bash
    pip install opencv-python-headless pymavlink
    ```

## Конфигурация

Перед запуском убедитесь, что все пути и параметры настроены правильно:

- `VIDEO_PATH` - Путь к видеофайлу.
- `DESIRED_WIDTH` и `DESIRED_HEIGHT` - Желаемые размеры видео.
- `COORDINATE_INTERVAL` - Интервал обновления координат.
- `FPS` - Кадры в секунду.
- `log_dir` - Директория для логов.

## Использование

### Запуск сервера

1. Запустите сервер для обмена данными через сокеты:

    ```bash
    python3 socket_server.py
    ```

    Сервер начнет слушать подключения на указанном хосте и порте.

### Запуск основного процесса

2. Запустите основной файл для обработки видео:

    ```bash
    python3 main_video.py
    ```

    Основной процесс захватывает видео, стабилизирует его, вычисляет координаты и отправляет данные на сервер.

## Компоненты

### main_video.py

Этот файл содержит главный процесс, который:

- Захватывает видео.
- Стабилизирует видео.
- Обрабатывает координаты.
- Отправляет данные на сервер сокетов.

Основные функции:

- `main()`: Основная функция, которая инициализирует видеопоток, стабилизирует его и вычисляет координаты объектов.

### support_positioning.py

Этот файл отвечает за:

- Инициализацию и настройку ключевых точек.
- Вычисление координат на основе видео.
- Асинхронное обновление опорного кадра.
- Получение данных азимута и высоты через MAVLink.

Основные функции:

- `setup(image, x_start, y_start)`: Инициализация опорного кадра.
- `calculate_coordinate(image, x_start, y_start)`: Вычисление координат на основе текущего кадра.
- `get_azimuth()`: Получение текущего азимута.
- `get_altitude()`: Получение текущей высоты.

### socket_server.py

Этот файл реализует сервер сокетов, который:

- Принимает и обрабатывает данные от клиентов.
- Отправляет координаты клиентам.

Основные функции:

- `start()`: Запуск сервера и обработка входящих подключений.
- `send_coordinates(x, y)`: Отправка координат всем подключенным клиентам.

## Логи

Все логи сохраняются в директорию, указанную в `log_dir`. Файлы логов включают:

- `flight_log.log`: Логи полета и вычислений координат.
- `server.log`: Логи сервера сокетов.