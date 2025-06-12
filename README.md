# Manipulator-MuJoCo-Env

This repository provides a MuJoCo-based simulation environment for robotic manipulators, developed using the `MjSpec` model construction and eliminate `dm_control` dependency. It is inspired by the original [Manipulator-MuJoCo](https://github.com/ian-chuang/Manipulator-Mujoco) project, but avoids the use of `dm_control`, offering a lighter and more flexible setup.

## Overview

The environment is designed with modularity in mind, allowing easy customization and extension of robot arms, grippers, control schemes, and simulation scenes. 

## Features

- UR5e 
- Robotiq 2F-85 
- MjSpec-based model construction (no `dm_control`)
- Gymnasium-compatible environment
- Operational Space Controller implementation
- Easily extendable architecture for custom robots, arenas, and tasks


## Installation

To get started, follow these steps to install the repository:

1. Clone this repository to your local machine:

   ```bash
   git clone https://github.com/abuibaid/Manipulator-Mujoco-Env
   ```

2. Navigate to the root directory of the repository:

   ```bash
   cd Manipulator-Mujoco-Env
   ```

3. Install the repository in editable mode:

   ```bash
   pip install -e .
   ```

## Demos

Explore the capabilities of Manipulator-Mujoco with the provided demos located in the `/demo` folder:


### UR5e Arm

To run the demo for the UR5e arm, execute:

```bash
python ur5e_demo.py
```

## Inspiration

This repository drew inspiration from the following repositories:
- [Manipulator-MuJoCo](https://github.com/ian-chuang/Manipulator-Mujoco)
- [ARISE-Initiative/robosuite.git](https://github.com/ARISE-Initiative/robosuite.git)
- [ir-lab/irl_control.git](https://github.com/ir-lab/irl_control.git)
- [abr/abr_control.git](https://github.com/abr/abr_control.git)
