# 🤖 Dynamic Obstacle Avoidance Robot

### Intelligent Robot Navigation using A* Search, Partial Observability and Dynamic Replanning

## 📌 Project Overview

The **Dynamic Obstacle Avoidance Robot** is a Python-based Artificial Intelligence project that simulates an autonomous robot navigating through a grid environment while avoiding static and dynamically moving obstacles.

The robot uses the **A* Search Algorithm** to calculate an optimal path from its starting position to the target. Unlike traditional pathfinding systems that assume complete knowledge of the environment, this project introduces **partial observability**, where the robot can only detect obstacles within a specified sensor range.

As obstacles move and new information becomes available, the robot updates its knowledge of the environment and recalculates its path to find a collision-free route.

The project includes both a command-line simulation and a graphical user interface built using Tkinter.

## 🎯 Objectives

- Implement intelligent robot navigation using the A* search algorithm.
- Avoid static and dynamically moving obstacles.
- Simulate partial observability using a limited sensor range.
- Maintain an updated representation of the known environment.
- Perform dynamic path replanning whenever obstacles affect the planned route.
- Demonstrate autonomous decision-making in an uncertain environment.

## 🧠 Core AI Concepts

### 1. A* Search Algorithm

A* is an informed search algorithm used to find a cost-effective path between the robot's current position and the target.

It evaluates nodes using:

**f(n) = g(n) + h(n)**

Where:
- `g(n)` — Actual cost from the starting position to the current node.
- `h(n)` — Estimated cost from the current node to the goal.
- `f(n)` — Total estimated path cost.

This project uses **Manhattan Distance** as its heuristic:

`h(n) = |x1 - x2| + |y1 - y2|`

The robot can move in four directions: up, down, left and right.

### 2. Partial Observability

The robot does not have complete knowledge of the environment.

Instead, it uses a simulated local sensor to detect obstacles within a specified Manhattan-distance range.

The environment contains:
- Known obstacles.
- Observed free cells.
- Unknown cells.
- Static obstacles.

The robot updates its internal map based on its current sensor observations.

### 3. Dynamic Obstacles

Dynamic obstacles change their positions during the simulation.

Their movement is randomized while ensuring that they do not move into:
- Static walls.
- The robot's current position.
- Other moving obstacles.

This introduces uncertainty into the navigation process.

### 4. Dynamic Path Replanning

A path calculated earlier may become unsuitable when an obstacle moves.

The robot therefore:
1. Observes its surrounding environment.
2. Updates its known obstacle map.
3. Executes A* search again.
4. Checks whether the next movement is safe in the simulated environment.
5. Moves or waits depending on the obstacle conditions.

This process continues until the robot reaches the target or the maximum step limit is reached.

## ⚙️ System Workflow

```text
        Initialize Grid Environment
                   |
                   v
        Set Robot Start and Goal
                   |
                   v
       Place Static and Moving Obstacles
                   |
                   v
          Simulate Sensor Detection
                   |
                   v
         Update Known Environment
                   |
                   v
            Apply A* Search
                   |
                   v
          Generate Optimal Path
                   |
                   v
        Check Next Movement Safety
                   |
             +-----+-----+
             |           |
             v           v
          Safe Move    Blocked
             |           |
             v           v
        Move Robot    Wait and Replan
             |           |
             +-----+-----+
                   |
                   v
        Update Environment Again
                   |
                   v
            Goal Reached?
             /       \
           Yes        No
            |          |
            v          v
         Success    Repeat Process
```

## ✨ Features

- **A* Pathfinding:** Calculates an optimal path using Manhattan distance.
- **Static Obstacle Avoidance:** Prevents navigation through fixed walls.
- **Dynamic Obstacle Simulation:** Models randomly moving obstacles.
- **Partial Observability:** Limits obstacle detection to the sensor range.
- **Dynamic Replanning:** Recalculates routes as the environment changes.
- **Collision Safety Checks:** Prevents movement into occupied cells in the simulated environment.
- **Custom Grid Configuration:** Allows users to define rows, columns, start position and target position.
- **Configurable Sensor Range:** Controls how far the robot can observe.
- **Step Limit:** Prevents unlimited simulation execution.

## 🖥️ Two Implementation Modes

### 1. Command-Line Simulation

File: `doar_foai.py`

The command-line version allows users to configure the environment through terminal inputs.

**Features:**
- Enter grid dimensions.
- Specify robot starting and target positions.
- Configure static and moving obstacles.
- Set sensor range and maximum steps.
- View the grid and planned path in the terminal.
- Monitor robot movements and obstacle detection.

### 2. Graphical User Interface

File: `dynamic_obstacle_avoidance_gui.py`

The GUI version uses Python Tkinter to provide an interactive visualization of robot navigation.

**Features:**
- Visual grid-based environment.
- Input fields for simulation configuration.
- Start Simulation button.
- Step Forward control.
- Stop and Reset controls.
- Real-time robot and obstacle visualization.
- Activity log and simulation status.
- Visual representation of the planned path.

## 🎨 Grid Legend

| Symbol | Description |
|---|---|
| R | Robot |
| G | Goal |
| # | Static obstacle / Wall |
| M | Detected moving obstacle |
| * | Planned path |
| ? | Unknown cell |
| . | Observed free cell (CLI) |

## 🛠️ Technologies Used

| Technology | Purpose |
|---|---|
| Python | Core programming language |
| Tkinter | Graphical user interface |
| Heapq | Priority queue for A* search |
| Random | Dynamic obstacle movement simulation |
| A* Search | Intelligent pathfinding |
| Manhattan Distance | Heuristic function |

## 📁 Project Structure

```text
Dynamic-Obstacle-Avoidance-Robot/
│
├── doar_foai.py
│   └── Command-line implementation
│
├── dynamic_obstacle_avoidance_gui.py
│   └── Tkinter graphical implementation
│
└── README.md
    └── Project documentation
```

## 🚀 Installation and Execution

### Prerequisites

- Python 3.x
- Tkinter (usually included with standard Python installations)

No external Python packages are required for the provided implementations.

### Step 1: Clone the Repository

```bash
git clone <YOUR_GITHUB_REPOSITORY_URL>
```

### Step 2: Navigate to the Project Folder

```bash
cd Dynamic-Obstacle-Avoidance-Robot
```

### Step 3: Run the Command-Line Version

```bash
python doar_foai.py
```

Enter the requested grid dimensions, starting position, goal position, obstacle locations, sensor range and maximum steps.

### Step 4: Run the GUI Version

```bash
python dynamic_obstacle_avoidance_gui.py
```

Configure the simulation settings in the left panel and use the available controls to execute the simulation.

## 🧪 Sample Configuration

The GUI implementation provides the following default configuration:

| Parameter | Value |
|---|---|
| Rows | 8 |
| Columns | 8 |
| Start Position | (0, 0) |
| Goal Position | (7, 7) |
| Static Obstacles | (2,2), (2,3), (4,4), (5,4) |
| Moving Obstacles | (1,5), (5,1) |
| Sensor Range | 2 |
| Maximum Steps | 100 |

**Obstacle input format:**

```text
2,3; 4,5; 5,2
```

Each position represents a row and column coordinate.

## 📈 Expected Output

The simulation displays:
- Current robot position.
- Target position.
- Detected moving obstacles.
- Planned navigation path.
- Path cost.
- Robot movement.
- Success or maximum-step termination status.

The simulation terminates successfully when the robot reaches the target.

If the robot cannot identify a valid path using its current map, it waits and observes the environment again.

## 🔮 Future Enhancements

- Integrate real-time camera or ultrasonic sensor data.
- Implement more advanced dynamic pathfinding algorithms.
- Introduce predictive obstacle movement models.
- Add diagonal robot movements.
- Implement physical robot hardware integration.
- Extend the simulation to larger and more complex environments.
- Introduce time-aware collision avoidance.

## 👥 Project Information

**Project Title:** Dynamic Obstacle Avoidance Robot

**Course:** CS23533 – Foundations of Artificial Intelligence

**Domain:** Artificial Intelligence | Autonomous Robotics | Intelligent Pathfinding

## 📚 Learning Outcomes

Through this project, we explored:

- Informed search algorithms.
- Heuristic-based pathfinding.
- Autonomous navigation.
- Partial observability in AI environments.
- Dynamic environment modelling.
- Collision avoidance.
- Real-time decision-making and replanning.
- Python GUI development.

---

**Built with Python and Artificial Intelligence concepts to demonstrate intelligent navigation in dynamic and partially observable environments.** 🤖
