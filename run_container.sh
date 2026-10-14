CONTAINER_NAME=reboost_dev
IMAGE_NAME=reboost_ubuntu24

xhost +local:docker

docker run -it \
    --rm \
    --name ${CONTAINER_NAME} \
    --gpus all \
    --network host \
    --device /dev/snd \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia:${LD_LIBRARY_PATH} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/workspace \
    ${IMAGE_NAME}