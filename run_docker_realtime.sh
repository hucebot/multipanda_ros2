#!/bin/bash

isRunning=`docker ps -f name=realtime_franka_humble | grep -c "realtime_franka_humble"`;

if [ $isRunning -eq 0 ]; then
    echo "Starting container with docker-compose..."
    xhost +local:docker
    docker compose -f docker-compose.yml up -d
    docker exec -it realtime_franka_humble /bin/bash
else
    echo "Container already running. Attaching..."
    docker exec -it realtime_franka_humble /bin/bash
fi