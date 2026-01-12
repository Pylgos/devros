#!/bin/bash

# Ensure the references directory exists
mkdir -p references

cd references

# Clone colcon-bash
if [ ! -d "colcon-bash" ]; then
  git clone https://github.com/colcon/colcon-bash.git colcon-bash
else
  echo "colcon-bash already exists, skipping clone."
fi

# Clone colcon-core
if [ ! -d "colcon-core" ]; then
  git clone https://github.com/colcon/colcon-core.git colcon-core
else
  echo "colcon-core already exists, skipping clone."
fi

# Clone colcon-ros
if [ ! -d "colcon-ros" ]; then
  git clone https://github.com/colcon/colcon-ros.git colcon-ros
else
  echo "colcon-ros already exists, skipping clone."
fi

# Clone colcon-cmake
if [ ! -d "colcon-cmake" ]; then
  git clone https://github.com/colcon/colcon-cmake.git colcon-cmake
else
  echo "colcon-cmake already exists, skipping clone."
fi

# Clone colcon-python-setup-py
if [ ! -d "colcon-python-setup-py" ]; then
  git clone https://github.com/colcon/colcon-python-setup-py.git colcon-python-setup-py
else
  echo "colcon-python-setup-py already exists, skipping clone."
fi

# Clone reps
if [ ! -d "reps" ]; then
  git clone https://github.com/openrobotics/reps.git reps
else
  echo "reps already exists, skipping clone."
fi

# Clone ament_cmake
if [ ! -d "ament_cmake" ]; then
  git clone https://github.com/ament/ament_cmake.git ament_cmake
else
  echo "ament_cmake already exists, skipping clone."
fi
