# pancakeBot

## Overview
This repository contains simulation and control code for a pancake-flipping task using a Baxter robot. The project was developed as a team-based final project for a Robotics course at Brigham Young University, in collaboration with Eli Smith and Thomas Murdoch.

The system was designed to support validation in simulation followed by deployment on physical hardware.

## My Contributions
My primary contributions focused on simulation:
- Developed the full environment and execution pipeline
- Implemented forward and inverse kinematics, including algorithms that account for both end-effector position and orientation
- Designed a path planning function that interpolates smoothly between joint-space poses
- Implemented obstacle avoidance logic to prevent collisions with the table and surrounding environment

## Methods
- Forward and Inverse Kinematics
- Path Planning
- Tools used (Python, Baxter Robot, Baxter ROS code, VScode)

## Results
- A project presentation and demonstration can be viewed here:
  https://www.youtube.com/watch?v=zeTX5xbyywE&list=WL&index=29
- The simulation successfully generated smooth motion between target poses
- Full program execution was successful, though performance was sensitive to the initial pancake placement
- Obstacle avoidance was effective at preventing collisions with the table; however, due to sensitivity introduced by joint limits in the inverse kinematics solution, it was not integrated into the main execution branch.

## How to Run Simulation
- Run main_sim.py

## How to Run Baxter Control
- Run main_baxter.py
