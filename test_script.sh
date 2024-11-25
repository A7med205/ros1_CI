#!/bin/bash
set -e  # Exit on error

# Variables
IMAGE_NAME="a7med205/ahmedhmair-cp22:tortoisebot-ros1-test"
CONTAINER_NAME="test_container"
SIMULATION_TIMEOUT=${SIMULATION_TIMEOUT:-120s} # Default timeout of 120s

# Print current directory and contents
pwd
ls -la

# Build Docker image
echo "Building Docker image..."
sudo docker build -t "$IMAGE_NAME" .

# Run container
echo "Starting container..."
sudo docker run -d --rm --name "$CONTAINER_NAME" \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "$IMAGE_NAME" bash -c \
  'source /catkin_ws/devel/setup.bash && roslaunch tortoisebot_gazebo tortoisebot_playground.launch'

# Wait for simulation readiness
echo "Waiting for simulation to initialize..."
sleep 15s

# Run test
echo "Running tests..."
sudo docker exec "$CONTAINER_NAME" bash -c \
  'source /catkin_ws/devel/setup.bash && rostest tortoisebot_waypoints waypoints_test.test --reuse-master'

# Cleanup: Kill simulation after timeout
echo "Simulation timeout: ${SIMULATION_TIMEOUT}..."
sleep "$SIMULATION_TIMEOUT" && sudo docker container kill "$CONTAINER_NAME" || true

# Print finish text
echo "Job finished successfully"
