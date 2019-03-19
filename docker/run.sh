#! /usr/bin/env bash

xhost +
docker-compose up --build
docker-compose down
xhost -