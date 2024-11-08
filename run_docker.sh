#!/bin/bash

device_options=""

if [ -e /dev/ttyUSB0 ]; then
    device_options+=" --device=/dev/ttyUSB0"
fi

if [ -e /dev/ttyACM0 ]; then
    device_options+=" --device=/dev/ttyACM0"
fi

docker run -it --rm \
    -v ./src/arm_control:/workspaces/src/arm_control \
    --network host \
    $device_options \
    --env-file ./.env \
    registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_base_image:latest \
    /bin/bash
