#!/usr/bin/env bash
# Дополнительный терминал в уже запущенном контейнере (окружение ROS уже подключено).
exec docker exec -it "${NAME:-dp_labs}" bash
