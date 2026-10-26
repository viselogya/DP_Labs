#!/usr/bin/env bash
# Запуск контейнера с выводом графики (Rviz, Gazebo, TurtleSim, rqt) на экран хоста.
# Использование (из корня репозитория):
#   ./docker/run.sh              - аппаратное ускорение через /dev/dri (Intel/AMD)
#   SOFTWARE_GL=1 ./docker/run.sh - программный рендеринг (если окна чёрные / Gazebo падает)
#   NVIDIA=1 ./docker/run.sh      - видеокарта NVIDIA (нужен nvidia-container-toolkit)
# Второй и следующие терминалы в том же контейнере: ./docker/exec.sh

IMAGE=${IMAGE:-dp_labs}
NAME=${NAME:-dp_labs}

# разрешить контейнеру подключаться к X-серверу хоста
xhost +local:root > /dev/null 2>&1 || echo "warning: xhost not found, GUI may not work"

ARGS=(-it --rm --name "$NAME" --net=host
      -e DISPLAY="$DISPLAY"
      -e QT_X11_NO_MITSHM=1
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw)

if [ -e /dev/dri ]; then
  ARGS+=(--device /dev/dri)
fi
if [ "${SOFTWARE_GL:-0}" = "1" ]; then
  ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi
if [ "${NVIDIA:-0}" = "1" ]; then
  ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

exec docker run "${ARGS[@]}" "$IMAGE" bash
