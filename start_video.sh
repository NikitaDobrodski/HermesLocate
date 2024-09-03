# #!/bin/bash

# LOG_FILE="/tmp/main_video.log"
# XVFB_LOG="/tmp/xvfb.log"

# # Запуск Xvfb
# Xvfb :99 -screen 0 1024x768x16 &> $XVFB_LOG &
# XVFB_PID=$!
# echo "Xvfb запущен с PID $XVFB_PID" | tee -a $LOG_FILE

# # Подождать, чтобы Xvfb успел запуститься
# sleep 15

# # Проверка, запустился ли Xvfb
# if ! ps -p $XVFB_PID > /dev/null; then
#     echo "Xvfb не удалось запустить" | tee -a $LOG_FILE
#     exit 1
# fi

# # Активировать виртуальное окружение
# source /home/adam/Documents/HermesLocate/venv/bin/activate

# # Установить переменную окружения для QT
# export QT_QPA_PLATFORM=xcb

# # Запуск main_video.py
# echo "Запуск main_video.py" | tee -a $LOG_FILE
# xvfb-run -a python /home/adam/Documents/HermesLocate/main_video.py &>> $LOG_FILE

# # Запись кода завершения работы
# EXIT_CODE=$?
# echo "main_video.py завершил работу с кодом $EXIT_CODE." | tee -a $LOG_FILE

# # Завершение Xvfb
# kill $XVFB_PID
# echo "Xvfb завершен." | tee -a $LOG_FILE

# exit $EXIT_CODE
