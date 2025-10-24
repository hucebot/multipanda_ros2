#!/bin/bash

isRunning=`docker ps -f name=sim_franka_humble | grep -c "sim_franka_humble"`;

if [ $isRunning -eq 0 ]; then
    echo "Starting container with docker-compose..."
    xhost +local:docker
    docker compose -f docker-compose-sim.yml up -d
    docker exec -it sim_franka_humble /bin/bash
else
    echo "Container already running. Attaching..."
    docker exec -it sim_franka_humble /bin/bash
fi