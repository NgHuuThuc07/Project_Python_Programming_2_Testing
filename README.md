# 🚗 MuJoCo Car Simulation — Rule-Based Obstacle Avoidance

> **Vietnamese-German University**  
> Course: Programming Methods 2 | Instructor: Bien Minh Tri | Major: Mechatronics

**Group Members:**
- Nguyễn Hữu Thức — 11525034
- Trần Đức Minh — 11525063
- Nguyễn Cảnh Nhật Nguyên — 11525051

---

## 📁 Project Structure

```
project/
├── car_controller.py       ← Main simulation controller
├── world.xml               ← MuJoCo scene (car, obstacles, ground)
├── Obstacle_Avoidance.puml ← PlantUML flowchart of avoidance logic

```

---

## ⚙️ Installation

```bash
pip install mujoco mujoco-python-viewer numpy matplotlib.
```
>Python ≥ 3.9, No GPU needed

---

## ▶️ How to Run

```bash
# With 3D viewer window
python car_controller.py
```

---

## 🗺️ Scene Overview

| Element                | Description                       |
|------------------------|-----------------------------------|
| 🟢 Green disc         | Start position `(-10.0, 0.0)`     |
| 🟡 Yellow disc + pole | Goal position `(10.0, 0.0)`       |
| 🔴 Red boxes          | 6 static obstacles and 1 wall     |
| 🔵 Blue car           | Autonomous vehicle with freejoint |
| 🟢 Green dot          | Front sensor indicator            |

**Obstacle positions (from `world.xml`):**

Blocks
| Name | X    | Y    | Size|
|------|------|------|-----|
| obs1 | -3.0 | +1.2 | 0.5 |
| obs2 | -1.0 | -1.0 | 0.5 |
| obs3 | +2.0 | +1.5 | 0.5 |
| obs4 | +3.0 | -0.3 | 0.5 |
| obs5 |  0.0 |  0.0 | 0.4 |
| obs6 |  5.0 |  2.0 | 0.6 |

Wall
| Name | X    | Y    | Size|
|------|------|------|-----|
| part1| 8.0  |  2.0 | 0.5 |
| part2| 8.0  |  1.0 | 0.5 |
| part3| 8.0  |  0.0 | 0.5 |
| part4| 8.0  | -1.0 | 0.5 |
| part5| 8.0  | -2.0 | 0.5 |

---

## 🔧 Configuration

All parameters are in one place — edit `car_controller.py`:

```python
@dataclass
class Config:
    start       = (-10.0, 0.0)   # Start position (x, y)
    goal        = ( 10.0, 0.0)   # Goal  position (x, y)

    obstacles   = [              # (x, y, half_size)
#Blocks
        (-3.0,  1.2, 0.5),
        (-1.0, -1.0, 0.5),
        ( 2.0,  1.5, 0.5),
        ( 3.0, -0.3, 0.5),
        ( 0.0,  0.0, 0.4),
        ( 5.0,  2.0, 0.6),
#Wall
        ( 8.0,  2.0, 0.5),
        ( 8.0,  1.0, 0.5),  
        ( 8.0,  0.0, 0.5), 
        ( 8.0, -1.0, 0.5),  
        ( 8.0, -2.0, 0.5)  
    ]

    forward_vel = 2.5   # m/s — straight speed
    turn_vel    = 1.5   # m/s — turning speed
    turn_rate   = 1.6   # rad/s — angular velocity
    dt          = 0.01  # s — physics timestep
    sense_range = 2.0   # m — sensor detection range
    sense_fov   = 0.55  # rad — side ray angle (~31.5°)
    goal_tol    = 0.55  # m — arrival radius
    sim_hz      = 40.0  # Hz — display speed (lower = slower)
```

> ⚠️ When changing **obstacle positions**, update **both** `obstacles` in `car_controller.py` **and** the `<geom>` positions in `world.xml` to stay in sync.

---

## 🧠 How It Works

### 1. Virtual Ray-Cast Sensor

Three rays are cast analytically from the car front:

```
ray_left   = yaw + 0.55 rad
ray_center = yaw + 0.00 rad
ray_right  = yaw - 0.55 rad

Returns: (dc, dl, dr) — distance to nearest obstacle per ray
Max range: 2.0 m
```

### 2. Obstacle Avoidance Logic (FSM)

```
Danger threshold: th = 2.0 × 0.52 = 1.04 m
```

```
┌─────────────────────────────────────────────────────┐
│  IF dc < 1.04m   (obstacle dead ahead)              │
│     IF dl >= dr  → AVOID_L  (turn left)             │
│     ELSE         → AVOID_R  (turn right)            │
│                                                     │
│  ELIF dl < 0.83m (clipping left wall)               │
│                  → AVOID_R  (steer right)           │
│                                                     │
│  ELIF dr < 0.83m (clipping right wall)              │
│                  → AVOID_L  (steer left)            │
│                                                     │
│  ELSE            → FORWARD  (go straight to goal)   │
└─────────────────────────────────────────────────────┘
```

### 3. Control Output per State

| State     | Speed (v) | Angular vel (ω)          | Description                   |
|-----------|-----------|--------------------------|-------------------------------|
| `FORWARD` | 2.5 m/s   | `clip(2.2 × herr, ±1.6)` | Straight + heading correction |
| `AVOID_L` | 1.5 m/s   | +1.6 rad/s               | Turn left                     |
| `AVOID_R` | 1.5 m/s   | −1.6 rad/s               | Turn right                    |
| `ARRIVED` | 0 m/s     | 0 rad/s                  | Stop at goal                  |

---

## 🔷 Flowchart (PlantUML)

**Flowchart summary:**
```
START
  └─ Car moves toward goal
       └─ [Loop] Scan 3 sensors
              ├─ Obstacle ahead?
              │     ├─ Left clear? → Turn LEFT
              │     └─ Right clear? → Turn RIGHT
              ├─ Near left wall? → Steer RIGHT
              ├─ Near right wall? → Steer LEFT
              └─ All clear? → Go STRAIGHT
       └─ Reached goal? → STOP
END
```

---

## 🌐 MuJoCo World (`world.xml`)

Key settings:

```xml
<option timestep="0.01" gravity="0 0 -9.81" integrator="RK4"/>
```

| Component | Type | Details |
|---|---|---|
| Ground | `plane` | 20×20 m, checker texture |
| Obstacles | `box` | 5 static red boxes |
| Car chassis | `box` | 0.84×0.44×0.16 m, blue |
| Wheels | `sphere` | 4 wheels, visual only |
| Sensor dot | `sphere` | Green, front of car |
| Physics joint | `freejoint` | 6 DOF free movement |

---
