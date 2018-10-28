#! /usr/bin/env bash

docker build -t calib-test .
xhost +
nvidia-docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -v `pwd`/ws:/root/ws --name calib-test-container --privileged --net host calib-test
xhost -