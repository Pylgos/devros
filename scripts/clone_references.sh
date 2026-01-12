#!/bin/bash

# Ensure the references directory exists
mkdir -p references

# Clone colcon-bash
if [ ! -d "references/colcon-bash" ]; then
  git clone https://github.com/colcon/colcon-bash.git references/colcon-bash
else
  echo "references/colcon-bash already exists, skipping clone."
fi

# Clone colcon-core
if [ ! -d "references/colcon-core" ]; then
  git clone https://github.com/colcon/colcon-core.git references/colcon-core
else
  echo "references/colcon-core already exists, skipping clone."
fi

# Clone colcon-ros
if [ ! -d "references/colcon-ros" ]; then
  git clone https://github.com/colcon/colcon-ros.git references/colcon-ros
else
  echo "references/colcon-ros already exists, skipping clone."
fi

# Clone reps
if [ ! -d "references/reps" ]; then
  git clone https://github.com/openrobotics/reps.git references/reps
else
  echo "references/reps already exists, skipping clone."
fi

# Clone ament_cmake
if [ ! -d "references/ament_cmake" ]; then
  git clone https://github.com/ament/ament_cmake.git references/ament_cmake
else
  echo "references/ament_cmake already exists, skipping clone."
fi
