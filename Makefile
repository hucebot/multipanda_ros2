.PHONY: build up access stop

# Build the images
build:
	docker compose build

# Start containers with xhost permissions
up:
	xhost +
	docker compose -f docker-compose.yml up

# Access the specific container shell
access:
	docker exec -it realtime_franka_humble bash

# Optional: Stop containers
stop:
	docker compose down