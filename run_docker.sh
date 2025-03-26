#!/bin/bash

xhost +local:docker
docker run -it --rm \
    --name moveit2_container \
    -v ./../../src:/workspaces/src \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /dev/shm:/dev/shm \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY=$DISPLAY \
    --network host \
    --ipc host \
    --pid host \
    --privileged \
    --env-file ./.env \
    registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_moveit_image:latest \
    /bin/bash
