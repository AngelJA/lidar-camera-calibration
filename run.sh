#! /usr/bin/env bash

xhost +
nvidia-docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -v ~/calib-test/ws:/root/ws --name calib-test-container --privileged --net host calib-test
xhost -