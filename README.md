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
└── output/                 ← Auto-generated on run
    ├── simulation_log.json
    ├── trajectory.png
    └── metrics.png
```

---

## ⚙️ Installation

```bash
pip install mujoco mujoco-python-viewer numpy matplotlib
```

> **Requirements:** Python ≥ 3.9, No GPU needed

---

## ▶️ How to Run

```bash
# With 3D viewer window
python car_controller.py

# Headless (no window, outputs plots only)
python car_controller.py --headless
```

---

## 🗺️ Scene Overview

| Element | Description |
|---|---|
| 🟢 Green disc | Start position `(-6.0, 0.0)` |
| 🟡 Yellow disc + pole | Goal position `(6.0, 0.0)` |
| 🔴 Red boxes | 5 static obstacles |
| 🔵 Blue car | Autonomous vehicle with freejoint |
| 🟢 Green dot | Front sensor indicator |

**Obstacle positions (from `world.xml`):**

| Name | X | Y | Size |
|---|---|---|---|
| obs1 | -3.0 | +1.2 | 0.5 |
| obs2 | -1.0 | -1.0 | 0.5 |
| obs3 | +2.0 | +1.5 | 0.5 |
| obs4 | +3.0 | -0.3 | 0.5 |
| obs5 |  0.0 |  0.0 | 0.4 |

---

## 🔧 Configuration (`Config` class)

All parameters are in one place — edit `car_controller.py`:

```python
@dataclass
class Config:
    start       = (-6.0, 0.0)   # Start position (x, y)
    goal        = ( 6.0, 0.0)   # Goal  position (x, y)

    obstacles   = [              # (x, y, half_size)
        (-3.0,  1.2, 0.5),
        (-1.0, -1.0, 0.5),
        ( 2.0,  1.5, 0.5),
        ( 3.0, -0.3, 0.5),
        ( 0.0,  0.0, 0.4),
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

> ⚠️ When changing **obstacle positions**, update **both** `Config.obstacles` in `car_controller.py` **and** the `<geom>` positions in `world.xml` to stay in sync.

---

## 🧠 How It Works

### System Architecture

```
┌─────────────────────────────────────────────┐
│              Simulation Loop                │
│                                             │
│  RayCastSensor  →  CarFSM  →  set_vel()    │
│   (3 ray scan)    (decide)   (apply cmd)   │
│        ↑                         ↓         │
│    get_state()          mujoco.mj_step()   │
│   (pos, yaw)              (physics dt)     │
└─────────────────────────────────────────────┘
```

### 1. Virtual Ray-Cast Sensor

Three rays are cast analytically from the car front using AABB slab intersection:

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
│  ELSE            → FORWARD  (go straight to goal)  │
└─────────────────────────────────────────────────────┘
```

### 3. Control Output per State

| State | Speed (v) | Angular vel (ω) | Description |
|---|---|---|---|
| `FORWARD` | 2.5 m/s | `clip(2.2 × herr, ±1.6)` | Straight + heading correction |
| `AVOID_L` | 1.5 m/s | +1.6 rad/s | Turn left |
| `AVOID_R` | 1.5 m/s | −1.6 rad/s | Turn right |
| `ARRIVED` | 0 m/s | 0 rad/s | Stop at goal |

---

## 🔷 Flowchart (PlantUML)

The file `Obstacle_Avoidance.puml` contains the flowchart of the avoidance logic. To render it:

**Option 1 — Online (no install):**
1. Go to [https://www.plantuml.com/plantuml](https://www.plantuml.com/plantuml)
2. Paste the content of `Obstacle_Avoidance.puml`
3. Export as PNG or SVG

**Option 2 — VS Code:**
Install the extension **PlantUML** by `jebbs`, then open the `.puml` file and press `Alt+D` to preview.

**Option 3 — Command line:**
```bash
pip install plantuml
plantuml Obstacle_Avoidance.puml
```

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

## 📊 Output Files

After simulation ends (close the viewer window):

| File | Description |
|---|---|
| `output/trajectory.png` | Top-down map of car path, colored by FSM state |
| `output/metrics.png` | Time-series: speed, angular rate, sensor distances |
| `output/simulation_log.json` | Raw data every step (pos, yaw, state, v, ω, sensors) |

**Trajectory color legend:**

| Color | State |
|---|---|
| 🔵 Blue | FORWARD |
| 🟠 Orange | AVOID_L |
| 🟢 Green | AVOID_R |
| 🟣 Purple | ARRIVED |

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

## 🔮 Possible Extensions

- **Multiple waypoints** — Replace single goal with a list of waypoints
- **Dynamic obstacles** — Give obstacles velocity in XML
- **Potential Field** — Upgrade FSM to gradient-based avoidance
- **Reinforcement Learning** — Wrap as Gymnasium env, train with Stable-Baselines3
- **Live viewer** — Enable `mujoco.viewer` with MuJoCo ≥ 3.0
