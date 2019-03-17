#! /usr/bin/env bash

xhost +
docker-compose up
docker-compose down
xhost -