# Manipulator-MuJoCo-Env

This repository provides a MuJoCo-based simulation environment for robotic manipulators, developed using the `MjSpec` interface. It is inspired by the original [Manipulator-MuJoCo](https://github.com/ian-chuang/Manipulator-Mujoco) project, but avoids the use of `dm_control`, offering a lighter and more flexible setup.

## Overview

The environment is designed with modularity in mind, allowing easy customization and extension of robot arms, grippers, control schemes, and simulation scenes. 

## Features

- UR5e robotic arm model
- Robotiq 2F-85 parallel gripper
- MjSpec-based simulation definition (no `dm_control`)
- Gymnasium-compatible environment
- Operational Space Controller implementation
- Easily extendable architecture for custom robots, arenas, and tasks


