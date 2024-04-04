<!-- markdownlint-disable MD024 -->

# My ME5413 Planning Project

> Authors: [Cao Chenyu](https://github.com/ruziniuuuuu), [Zhao Xu](https://github.com/AeroEmbedAutoTechJohn), [Li Zhangjin](https://github.com/Lizhangjin)

This is my forked version of the [ME5413_PLANNING_Project](https://github.com/ruziniuuuuu/ME5413_Planning_Project_Group20.git).

## Introduction

This project is our group's work on implementaing planning algorithms for autonomous models. There are three tasks:

- Task 1: Implementing the A* algorithm
- Task 2: Solving the Traveling Salesman Problem (TSP)
- Bonus Task: Implementing the Path path tracker controller to follow a figure-8 reference trajectory

## Installation and Execution

## Task 1 & 2

Create a virtual environment and install the required packages in the `requirements.txt` file.

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Then, run the `homework3.ipynb` notebook to see all the implementations and results.

## Task 3 (Bonus)

First, follow the instructions in the [ORIGINAL_README](./README_ORIGINAL.md) to install the required packages.

Next, run the following commands in one terminal to set up the Gazebo environment:

```bash
roslaunch me5413_world world.launch
```

Then, run the following commands in another terminal to run the controller, there are three controllers to choose from:

- `pid_stanley_original`
- `pid_stanley_improved`
- `pid_purepursuit`

```bash
roslaunch me5413_world path_tracking_pid_stanley_original.launch
# rosrun me5413_world path_tracking_pid_stanley_improved.launch
# rosrun me5413_world path_tracking_pid_purepursuit.launch
```

## Project Structure

```plaintext
ME5413_Planning_Project_Group20
├─ HW3_Task1&2
│  ├─ LICENSE
│  ├─ map
│  ├─ results
│  └─ src
│     └─ extract_free_space.py
│  └─ homework3.ipynb
├─ outputs
│  ├─ pid_purepursuit
│  ├─ pid_stanley_improved
│  └─ pid_stanley_original
├─ report_typ
│  ├─ main.typ
│  ├─ main.pdf
│  └─ refs.bib
└─ src
   ├─ jackal_description
   └─ me5413_world
      ├─ cfg
      ├─ include
      │  └─ me5413_world
      ├─ launch
      │  └─ include
      ├─ media
      ├─ rviz
      ├─ scripts
      | ├─ tracker_error_logger.py
      ├─ src
      | ├─ path_tpublisher_node.py
      | ├─ path_tracker_pid_purepursuit_node.py
      | ├─ path_tracker_pid_stanley_improved_node.py
      | ├─ path_tracker_pid_stanley_original_node.py
      └─ worlds
```

## TODO List

### Task 1: Graph Search Algorithms

- [x] Implement the A* algorithm
- [x] Degenrate the A* algorithm to Dijkstra's algorithm
- [x] Degenrate the A* algorithm to Greedy Best First Search
- [x] Compare the performance of the three algorithms in terms of path length, nodes visited, and computation time
- [x] Implement the A* algorithm with a different heuristic (Manhattan distance)

### Task 2: Traveling Salesman Problem

- [x] Implement the brute-force algorithm
- [x] Implement the Dynamic Programming algorithm
- [x] Implement the Genetic Algorithm
- [x] Compare the performance of the three algorithms in terms of path length, nodes visited, and computation time

### Bonus Task: Path Tracking Controller

- [x] Run the original PID + Stanley Controller and tune the parameters dynamically
- [x] Improve the basic Stanley Controller by adjusting its gain based on the current velocity
- [x] Implemene the Pure Pursuit Controller
- [x] Evaluate and compare the three controllers in terms of `RMS Position Error`, `RMS Heading Error`, and `RMS Speed Error`
