isRunning=`docker ps -f name=sim_franka_humble | grep -c "sim_franka_humble"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm sim_franka_humble
    docker run  \
        -it \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
        --env="ROS_DOMAIN_ID=39" \
        --name=sim_franka_humble \
        --network="host" \
        --entrypoint /bin/bash \
        hucebot:franka-humble

else
    echo "Docker already running."
    docker exec -it sim_franka_humble /bin/bash
fi
