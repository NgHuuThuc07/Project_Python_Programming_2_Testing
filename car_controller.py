"""
MuJoCo Car Simulation — Rule-Based Obstacle Avoidance

> **Vietnamese-German University**  
> Course: Programming Methods 2 | Instructor: Bien Minh Tri | Major: Mechatronics

**Group Members:**
- Nguyễn Hữu Thức — 11525034
- Trần Đức Minh — 11525063
- Nguyễn Cảnh Nhật Nguyên — 11525051

"""
import argparse, os, sys, time, json
import numpy as np
import mujoco
from dataclasses import dataclass, field
from typing import List, Tuple

_HERE   = os.path.dirname(os.path.abspath(__file__))
_XML    = os.path.join(_HERE, "world.xml")          # same folder as .py

# ── Config ────────────────────────────────────────────────────
@dataclass
class Config:
    xml_path    : str   = _XML
    start       : tuple = (-10.0, 0.0)
    goal        : tuple = ( 10.0, 0.0)
    obstacles   : list  = field(default_factory=lambda: [
        (-3.0,  1.2, 0.5), (-1.0, -1.0, 0.5),
        ( 2.0,  1.5, 0.5), ( 3.0, -0.3, 0.5),  #replace by  ( 1.0,  1.5, 0.5),( 3.0, -1.3, 0.5) to have new direction for car
        ( 0.0,  0.0, 0.4), ( 5.0,  2.0, 0.6),

        #Wall
        ( 8.0,  2.0, 0.5),  
        ( 8.0,  1.0, 0.5),  
        ( 8.0,  0.0, 0.5),  
        ( 8.0, -1.0, 0.5),  
        ( 8.0, -2.0, 0.5)   
    ])
    forward_vel : float = 2.5
    turn_vel    : float = 1.5
    turn_rate   : float = 1.6
    dt          : float = 0.01
    sense_range : float = 2.0
    sense_fov   : float = 0.55
    goal_tol    : float = 0.55
    max_steps   : int   = 10_000
    log_every   : int   = 100
    sim_hz      : float = 40.0   # real-time speed (steps/sec)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ── Ray-Cast Sensor ───────────────────────────────────────────
class RayCastSensor:
    def __init__(self, obs, rng, fov):
        self.obs  = obs
        self.rng = rng
        self.rays = [0.0, fov, -fov]   # center, left, right

    def _hit(self, pos, angle, ox, oy, hs):
        d = np.array([np.cos(angle), np.sin(angle)])
        e = 1e-12
        txlo = (ox - hs - pos[0]) / (d[0]+e); txhi = (ox + hs - pos[0]) / (d[0]+e)
        if txlo > txhi: txlo, txhi = txhi, txlo
        tylo = (oy - hs - pos[1]) / (d[1]+e); tyhi = (oy + hs - pos[1]) / (d[1]+e)
        if tylo > tyhi: tylo, tyhi = tyhi, tylo
        t0, t1 = max(txlo, tylo), min(txhi, tyhi)
        if t1 < 0 or t0 > t1 or t0 > self.rng: return float("inf")
        return max(0.0, t0)

    def scan(self, pos, yaw):
        out = []
        for off in self.rays:
            a = yaw + off; best = self.rng
            for ox, oy, hs in self.obs:
                h = self._hit(pos, a, ox, oy, hs)
                if h < best: best = h
            out.append(best)
        return tuple(out)   # center, left, right


# ── FSM ───────────────────────────────────────────────────────
class CarFSM:
    FORWARD="FORWARD"; AVOID_L="AVOID_L"; AVOID_R="AVOID_R"; ARRIVED="ARRIVED"

    def __init__(self, cfg):
        self.cfg = cfg; self.state = self.FORWARD

    def step(self, dc, dl, dr, herr):
        cfg = self.cfg; th = cfg.sense_range * 0.52
        if self.state == self.ARRIVED: return 0.0, 0.0
        if   dc < th:        self.state = self.AVOID_L if dl >= dr else self.AVOID_R
        elif dl < th * 0.80: self.state = self.AVOID_R
        elif dr < th * 0.80: self.state = self.AVOID_L
        else:                self.state = self.FORWARD
        if   self.state == self.FORWARD:
            return cfg.forward_vel, float(np.clip(2.2*herr, -cfg.turn_rate, cfg.turn_rate))
        elif self.state == self.AVOID_L: return cfg.turn_vel, +cfg.turn_rate
        elif self.state == self.AVOID_R: return cfg.turn_vel, -cfg.turn_rate
        return 0.0, 0.0


# ── Physics ───────────────────────────────────────────────────
def get_state(data):
    pos = data.sensor("car_pos").data[:2].copy()
    xax = data.sensor("car_xaxis").data[:2]
    return pos, float(np.arctan2(xax[1], xax[0]))

def set_vel(data, model, v, omega, yaw):
    jid = model.joint("car_freejoint").id
    qa  = model.jnt_dofadr[jid]
    data.qvel[qa+0]=v*np.cos(yaw); data.qvel[qa+1]=v*np.sin(yaw)
    data.qvel[qa+2]=data.qvel[qa+3]=data.qvel[qa+4]=0.0
    data.qvel[qa+5]=omega

def reset_car(data, model, cfg):
    jid = model.joint("car_freejoint").id
    qa  = model.jnt_qposadr[jid]
    data.qpos[qa:qa+7] = [cfg.start[0], cfg.start[1], 0.12, 1, 0, 0, 0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


# ── Logger ────────────────────────────────────────────────────
class Logger:
    def __init__(self): self.rows = []
    def push(self, step, pos, yaw, state, dists, herr, v, omega):
        self.rows.append(dict(step=step, x=float(pos[0]), y=float(pos[1]),
            yaw=float(yaw), state=state, dc=float(dists[0]),
            dl=float(dists[1]), dr=float(dists[2]),
            herr=float(herr), v=float(v), omega=float(omega)))
    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path,"w") as f: json.dump(self.rows, f, indent=2)
        print(f"  [LOG] {len(self.rows)} steps -> {path}")


# ── Viewer launcher ───────────────────────────────────────────
def try_launch_viewer(model, data):
    """
    Thu tu: mujoco.viewer (v3.x) -> mujoco_viewer -> OpenCV fallback
    Tra ve (viewer_obj, mode_string)
    """
    # 1) MuJoCo built-in viewer (>= 3.x)
    try:
        import mujoco.viewer as _mv
        viewer = _mv.launch_passive(model, data)
        viewer.cam.lookat[:] = [0, 0, 0]
        viewer.cam.distance  = 30.0
        viewer.cam.azimuth   = -90.0
        viewer.cam.elevation = -35.0
        print("  [VIEWER] mujoco.viewer (built-in) -- OK")
        return viewer, "builtin"
    except Exception as e:
        print(f"  [INFO] mujoco.viewer not available: {e}")

    # 2) mujoco-python-viewer package
    try:
        import mujoco_viewer
        viewer = mujoco_viewer.MujocoViewer(model, data,
                                            title="Car Simulation",
                                            width=1280, height=720)
        print("  [VIEWER] mujoco_viewer (mujoco-python-viewer) -- OK")
        return viewer, "mjpv"
    except ImportError:
        print("  [INFO] mujoco_viewer not installed.")
        print("         -> pip install mujoco-python-viewer")
    except Exception as e:
        print(f"  [INFO] mujoco_viewer error: {e}")

    return None, "headless"


# ── Main ──────────────────────────────────────────────────────
def run(cfg: Config, headless=False):
    print("\n" + "="*58)
    print("  MuJoCo Car Simulation  |  Rule-Based Avoidance")
    print("="*58)

    xml_path = os.path.abspath(cfg.xml_path)
    if not os.path.exists(xml_path):
        sys.exit(f"[ERROR] XML not found: {xml_path}")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)
    reset_car(data, model, cfg)

    sensor  = RayCastSensor(cfg.obstacles, cfg.sense_range, cfg.sense_fov)
    fsm     = CarFSM(cfg)
    logger  = Logger()
    goal    = np.array(cfg.goal, dtype=float)
    arrived = False
    step    = 0

    # ── Detect viewer ─────────────────────────────────────────
    viewer, mode = (None, "headless") if headless else try_launch_viewer(model, data)

    print(f"  Mode      : {'HEADLESS' if mode=='headless' else 'VIEWER 3D (' + mode + ')'}")
    print(f"  Start     : {cfg.start}  ->  Goal: {cfg.goal}")
    print(f"  Obstacles : {len(cfg.obstacles)}")
    print("="*58 + "\n")
    if mode != "headless":
        print("  [VIEWER] Cua so 3D da mo. Dong cua so de ket thuc.\n")

    step_time = 1.0 / cfg.sim_hz

    for _ in range(cfg.max_steps):
        t0 = time.perf_counter()

        # ── Sense ───────────────────────────────────────────
        pos, yaw = get_state(data)
        to_goal  = goal - pos
        d_goal   = float(np.linalg.norm(to_goal))
        herr     = wrap(np.arctan2(to_goal[1], to_goal[0]) - yaw)

        # ── Goal check ──────────────────────────────────────
        if d_goal < cfg.goal_tol:
            fsm.state = CarFSM.ARRIVED
            set_vel(data, model, 0.0, 0.0, yaw)
            logger.push(step, pos, yaw, fsm.state, (0.,0.,0.), 0., 0., 0.)
            arrived = True
            print(f"\n  *** GOAL REACHED  step={step}  t={step*cfg.dt:.2f}s ***\n")
            # Giu nguyen 3 giay trong viewer
            if viewer and mode == "builtin":
                for _ in range(1000):
                    try: viewer.sync(); time.sleep(0.01)
                    except: break
            elif viewer and mode == "mjpv":
                for _ in range(1000):
                    try:
                        if not viewer.is_alive: break
                        viewer.render(); time.sleep(0.01)
                    except: break
            break

        # ── Plan & Act ──────────────────────────────────────
        dists    = sensor.scan(pos, yaw)
        v, omega = fsm.step(*dists, herr)
        set_vel(data, model, v, omega, yaw)
        mujoco.mj_step(model, data)
        logger.push(step, pos, yaw, fsm.state, dists, herr, v, omega)

        if step % cfg.log_every == 0:
            print(f"  step={step:5d}  t={step*cfg.dt:6.2f}s  "
                  f"pos=({pos[0]:+6.2f},{pos[1]:+6.2f})  "
                  f"yaw={np.degrees(yaw):+6.1f}deg  "
                  f"state={fsm.state:<8}  d2goal={d_goal:.2f}m  "
                  f"DC={dists[0]:.2f} DL={dists[1]:.2f} DR={dists[2]:.2f}")

        # ── Render ──────────────────────────────────────────
        if mode == "builtin":
            try:
                if not viewer.is_running(): break
                viewer.sync()
            except: break
        elif mode == "mjpv":
            try:
                if not viewer.is_alive: break
                viewer.render()
            except: break

        # ── Real-time pacing ────────────────────────────────
        if mode != "headless":
            slp = step_time - (time.perf_counter() - t0)
            if slp > 0: time.sleep(slp)

        step += 1

    if not arrived:
        print(f"\n  Max steps {cfg.max_steps} reached.\n")

    if viewer:
        try: viewer.close()
        except: pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Khong mo cua so 3D")
    args = parser.parse_args()
    run(Config(), headless=args.headless)
