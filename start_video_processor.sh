#!/bin/bash


Xvfb :99 -screen 0 1024x768x16 &> /tmp/xvfb.log &
XVFB_PID=$!
echo "Xvfb запущен с PID $XVFB_PID" | tee -a /tmp/main_video.log


sleep 15


if ! ps -p $XVFB_PID > /dev/null; then
    echo "Xvfb failed to start" | tee -a /tmp/main_video.log
    exit 1
fi


source /home/adam/Documents/HermesLocate/venv/bin/activate


export QT_QPA_PLATFORM=xcb


echo "Запуск main_video.py" | tee -a /tmp/main_video.log
xvfb-run -a python /home/adam/Documents/HermesLocate/main_video.py &>> /tmp/main_video.log


echo "main_video.py завершил работу с кодом $?." | tee -a /tmp/main_video.log


kill $XVFB_PID
echo "Xvfb завершен." | tee -a /tmp/main_video.log

