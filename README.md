# Visual-Inertial-Odometry
VIO algorithm using stereo camera and IMU data within ROS2 environment.

## Workflow
This project follows a 5-step workflow: Data Input, Initialization, Stereo VIO, Loop Closure, and Global Optimization.
![workflow](assets/workflow_flowchart.jpg)

## System Architecture
![flowchart](assets/VINS-FUSION-FLOWCHART.jpg)

## What it does
Augmenting the VINS-Fusion algorithm with learning-based Feature Selection and Adaptive NMS

## How I built
I implemented a five-step ROS2 pipeline based on VINS-Fusion to process sensor data and generate an optimized trajectory for evaluation.

## What I learned
I learned the fundamentals of visual-inertial sensor fusion, the importance of loop closure in reducing drift, and how to benchmark VIO systems.
