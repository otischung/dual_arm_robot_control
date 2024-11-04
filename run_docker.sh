#!/bin/bash

docker run -it --rm \
    -v ./src:/workspaces/src \
    registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_base_image:latest \
    /bin/bash
