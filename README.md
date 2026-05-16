# Frontier-Based Exploration Simulator

A 2D frontier-based exploration and occupancy grid mapping simulator built with Python + Pygame.

This project simulates a simplified autonomous exploration pipeline commonly used in SLAM (Simultaneous Localization and Mapping) systems, including:

- Occupancy Grid Mapping
- Frontier Detection
- Wavefront-based Frontier Exploration
- LiDAR-like Raycasting Sensor
- Path Planning and Path Following
- Policy-based Exploration Framework

---

# Features

## Occupancy Grid Mapping

- Unknown / Free / Obstacle map representation
- Dynamic occupancy map updates
- Hidden ground-truth environment
- Random map generation

## Frontier-Based Exploration

- Frontier extraction from occupancy map
- Wavefront distance cost calculation
- Information gain based frontier selection
- Autonomous exploration policy

## LiDAR-like Sensor Simulation

- Raycasting-based sensing
- Obstacle occlusion
- Configurable:
  - Sensor range
  - Ray count
  - Ray marching precision
  - Field of view (FOV)

## Robot Simulation

- Continuous 2D movement
- Heading-based LiDAR rotation
- Circular collision detection
- Sliding collision response

## Exploration Policy Framework

Current supported policies:

- Manual Policy
- Random Walk Policy
- LPFE-like Frontier Exploration Policy

The framework is designed for future extensions such as:

- Multi-Robot Exploration
- Utility-Based Exploration
- RRT Exploration
- Semantic Exploration
- RL-based Exploration

---

# Installation

```bash
pip install -r requirements.txt
```

---

# Run

## Manual Exploration

```bash
python main.py
```

or

```bash
python main.py -p manual
```

## Random Walk Exploration

```bash
python main.py -p random
```

## LPFE-like Frontier Exploration

```bash
python main.py -p lpfe
```

---

# Controls

| Key | Action |
|---|---|
| `W A S D` | Move robot |
| `Arrow Keys` | Move robot |
| `R` | Regenerate map |
| `ESC` | Quit |

---

# Visualization

| Color | Meaning |
|---|---|
| Dark Gray | Unknown |
| Light Gray | Free Space |
| Black | Obstacle |
| Yellow | Frontier |
| Red Rays | LiDAR Rays |
| Blue Circle | Robot |

---

# LiDAR Configuration

Configurable in `config.py`:

```python
SENSOR_RADIUS
LIDAR_RAY_COUNT
LIDAR_STEP_SIZE
LIDAR_FOV_DEG
```

Example:

```python
SENSOR_RADIUS = 8
LIDAR_RAY_COUNT = 180
LIDAR_STEP_SIZE = 0.15
LIDAR_FOV_DEG = 270
```

---

# Project Structure

```text
project/
├── main.py
├── config.py
├── world.py
├── occupancy_map.py
├── robot.py
├── renderer.py
├── frontier.py
│
├── exploration/
│   ├── base.py
│   ├── manual.py
│   ├── random_walk.py
│   └── lpfe.py
│
└── requirements.txt
```

---

# Exploration Pipeline

```text
LiDAR Sensor
    ↓
Occupancy Grid Mapping
    ↓
Frontier Detection
    ↓
Wavefront Cost Calculation
    ↓
Target Frontier Selection
    ↓
Path Reconstruction
    ↓
Robot Navigation
```

---

# Current Limitations

- Single robot only
- No probabilistic occupancy update
- No SLAM uncertainty model
- No dynamic obstacles
- Simplified local planner
- Grid-based wavefront planner only

---

# Future Work

- Multi-robot exploration
- Distributed frontier assignment
- Loop closure simulation
- Dynamic obstacle avoidance
- ROS2 integration
- Real SLAM backend integration
- Information-theoretic exploration
- RL-based exploration policies