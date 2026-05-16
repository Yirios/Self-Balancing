"""3D visualization of the self-balancing robot with path trail."""
import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import gymnasium as gym
import balancing_robot
from stable_baselines3 import PPO
from pretrain_bc import BCModel
from balancing_robot.dynamics import R, L_1, L_2


WHEEL_BASE = 0.16   # meters between wheels
WHEEL_WIDTH = 0.02  # wheel thickness (visual)
BODY_W = 0.04        # body half-width


def _robot_vertices(x, y, yaw, theta_1, theta_2):
    """Return 3D line/point coords for the robot at given pose."""
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)

    def transform(rx, ry, rz):
        """Rotate by yaw around Z, then translate."""
        wx = rx * cos_y - ry * sin_y + x
        wy = rx * sin_y + ry * cos_y + y
        wz = rz
        return np.array([wx, wy, wz])

    # Axle center
    axle = transform(0, 0, R)

    # Wheels
    left_wheel = transform(0, -WHEEL_BASE / 2, R)
    right_wheel = transform(0, WHEEL_BASE / 2, R)

    # Left wheel circle points
    n_pts = 12
    angles = np.linspace(0, 2 * np.pi, n_pts, endpoint=False)
    lw = np.array([
        transform(
            0,
            -WHEEL_BASE / 2 + WHEEL_WIDTH * np.cos(a),
            R + WHEEL_WIDTH * np.sin(a),
        )
        for a in angles
    ])
    rw = np.array([
        transform(
            0,
            WHEEL_BASE / 2 + WHEEL_WIDTH * np.cos(a),
            R + WHEEL_WIDTH * np.sin(a),
        )
        for a in angles
    ])

    # Body: from axle to top, tilted by theta_1 in XZ plane
    body_top_rx = L_1 * np.sin(theta_1)
    body_top_rz = R + L_1 * np.cos(theta_1)
    body_top = transform(body_top_rx, 0, body_top_rz)

    # Pendulum: absolute angle from vertical (theta_2 is absolute, not relative)
    pend_angle = theta_2
    pend_top_rx = body_top_rx + L_2 * np.sin(pend_angle)
    pend_top_rz = body_top_rz + L_2 * np.cos(pend_angle)
    pend_top = transform(pend_top_rx, 0, pend_top_rz)

    # Body box (narrow cuboid)
    hw = BODY_W  # half-width
    box = np.array([
        transform(-hw, -hw, R),
        transform(-hw, hw, R),
        transform(hw, hw, body_top_rz),
        transform(hw, -hw, body_top_rz),
    ])

    return axle, left_wheel, right_wheel, lw, rw, body_top, pend_top, box


def run_and_record(
    model_type: str,
    deterministic: bool = True,
    steps: int = 500,
    drive: tuple[float, float] = (0.0, 0.0),
    env_kwargs: dict | None = None,
):
    """Run episode, record states and global positions for animation.

    drive = (target_theta_L, target_theta_R):  LQR position targets (rad)
    env_kwargs: passed to gym.make (e.g. use_pi_motor, motor_gain)
    """
    if env_kwargs:
        env = gym.make("BalancingRobot-v0", **env_kwargs)
    else:
        env = gym.make("BalancingRobot-v0")

    if model_type == "bc":
        bc = BCModel()
        bc.load_state_dict(torch.load("bc_model.pt", map_location="cpu"))
        bc.eval()

        def policy(obs):
            return bc(torch.from_numpy(obs.astype(np.float32))).detach().numpy()

        label = "BC (LQR imitation)"
    elif model_type == "ppo":
        model = PPO.load("best_model_reg/best_model", device="cpu")

        def policy(obs):
            return model.predict(obs, deterministic=deterministic)[0]

        label = f"BC-reg PPO ({'deterministic' if deterministic else 'stochastic'})"
    elif model_type == "lqr":
        from balancing_robot.dynamics import get_lqr_gains
        K = get_lqr_gains()

        def policy(obs):
            # obs[0:2] are already (thL-targetL, thR-targetR)
            pos_err = obs[0:2]
            x = np.array([pos_err[0], pos_err[1], obs[2], obs[3],
                          obs[4], obs[5], obs[6], obs[7]])
            x_ref = np.zeros(8)
            return -K @ (x - x_ref)

        label = "LQR"
    else:
        raise ValueError(f"Unknown model: {model_type}")

    target_L, target_R = drive  # position targets (rad)
    if target_L != 0 or target_R != 0:
        label += f"  targets: L={target_L:.1f} R={target_R:.1f}"

    print(f"Recording: {label}")
    obs, _ = env.reset(seed=42, options={"target_theta_L": target_L, "target_theta_R": target_R})
    # Start from near-equilibrium (small tilt, zero velocity)
    if env_kwargs and (env_kwargs.get("use_pi_motor") or env_kwargs.get("data_driven")):
        env.unwrapped.state = np.zeros(8)
        env.unwrapped.state[0] = target_L
        env.unwrapped.state[1] = target_R
        env.unwrapped.state[2] = 0.02  # ~1.15° tilt
    else:
        env.unwrapped.state[0] = target_L
        env.unwrapped.state[1] = target_R
    obs = env.unwrapped._get_obs()

    states, xs, ys, yaws = [], [], [], []
    for i in range(steps):
        # Pendulum impulses at 1s, 2.5s, 4s
        if i in (100, 250, 400):
            env.unwrapped.state[7] += 0.8  # theta_dot_2 impulse (matches real car th2 range)
        u = policy(obs)
        obs, _, terminated, truncated, _ = env.step(u)
        states.append(obs.copy())
        xs.append(env.unwrapped.x)
        ys.append(env.unwrapped.y)
        yaws.append(env.unwrapped.yaw)
        if terminated:
            break

    env.close()
    print(f"  Recorded {len(states)} frames")
    return states, xs, ys, yaws, label


def make_animation_3d(states, xs, ys, yaws, label, filename="balance_3d.gif"):
    """Create 3D matplotlib animation and save as GIF."""
    fig = plt.figure(figsize=(14, 6))
    ax = fig.add_subplot(121, projection="3d")
    ax_plot = fig.add_subplot(122)

    # --- 3D view ---
    margin = 0.4
    all_x, all_y = np.array(xs), np.array(ys)
    x_min, x_max = all_x.min() - margin, all_x.max() + margin
    y_min, y_max = all_y.min() - margin, all_y.max() + margin
    x_mid, y_mid = (x_min + x_max) / 2, (y_min + y_max) / 2
    rng = max(x_max - x_min, y_max - y_min, 0.5)

    ax.set_xlim(x_mid - rng / 2, x_mid + rng / 2)
    ax.set_ylim(y_mid - rng / 2, y_mid + rng / 2)
    ax.set_zlim(0, 0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(label)

    # Ground plane
    xx, yy = np.meshgrid(
        np.linspace(x_mid - rng / 2, x_mid + rng / 2, 6),
        np.linspace(y_mid - rng / 2, y_mid + rng / 2, 6),
    )
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.1, color="gray")

    # Path trail
    (trail_line,) = ax.plot([], [], [], "k--", linewidth=0.5, alpha=0.5)

    # Robot elements (initialized empty, updated each frame)
    (axle_pt,) = ax.plot([], [], [], "ko", markersize=6)
    (body_ln,) = ax.plot([], [], [], color="#c83232", linewidth=4)
    (pend_ln,) = ax.plot([], [], [], color="#3232c8", linewidth=2)
    (body_pt,) = ax.plot([], [], [], "o", color="#c83232", markersize=8)
    (pend_pt,) = ax.plot([], [], [], "o", color="#3232c8", markersize=6)

    lw_lines = [ax.plot([], [], [], "k-", linewidth=2)[0] for _ in range(12)]
    rw_lines = [ax.plot([], [], [], "k-", linewidth=2)[0] for _ in range(12)]
    (axle_bar,) = ax.plot([], [], [], "k-", linewidth=3)

    # --- Angle plot ---
    ax_plot.set_xlim(0, len(states))
    ax_plot.set_ylim(-15, 15)
    ax_plot.set_xlabel("Step")
    ax_plot.set_ylabel("Angle (°)")
    ax_plot.set_title("Body & Pendulum Angles")
    ax_plot.axhline(y=0, color="gray", linewidth=0.5)
    (ln_t1,) = ax_plot.plot([], [], "r-", label=r"$\theta_1$ (body)", linewidth=1)
    (ln_t2,) = ax_plot.plot([], [], "b-", label=r"$\theta_2$ (pendulum)", linewidth=1)
    ax_plot.legend(loc="upper right")

    theta1_hist = [s[2] for s in states]
    theta2_hist = [s[3] for s in states]

    def update(i):
        s = states[i]
        x, y, yaw = xs[i], ys[i], yaws[i]
        theta_1, theta_2 = s[2], s[3]

        axle, lw, rw, lwp, rwp, btop, ptop, box = _robot_vertices(
            x, y, yaw, theta_1, theta_2
        )

        # Path trail up to current frame
        trail_line.set_data(np.array(xs[: i + 1]), np.array(ys[: i + 1]))
        trail_line.set_3d_properties(np.zeros(i + 1))

        # Axle
        axle_pt.set_data([axle[0]], [axle[1]])
        axle_pt.set_3d_properties([axle[2]])

        # Axle bar (left wheel to right wheel)
        axle_bar.set_data([lw[0], rw[0]], [lw[1], rw[1]])
        axle_bar.set_3d_properties([lw[2], rw[2]])

        # Left wheel outline
        for k in range(12):
            nxt = (k + 1) % 12
            lw_lines[k].set_data(
                [lwp[k, 0], lwp[nxt, 0]], [lwp[k, 1], lwp[nxt, 1]]
            )
            lw_lines[k].set_3d_properties([lwp[k, 2], lwp[nxt, 2]])
            rw_lines[k].set_data(
                [rwp[k, 0], rwp[nxt, 0]], [rwp[k, 1], rwp[nxt, 1]]
            )
            rw_lines[k].set_3d_properties([rwp[k, 2], rwp[nxt, 2]])

        # Body line
        body_ln.set_data([axle[0], btop[0]], [axle[1], btop[1]])
        body_ln.set_3d_properties([axle[2], btop[2]])
        body_pt.set_data([btop[0]], [btop[1]])
        body_pt.set_3d_properties([btop[2]])

        # Pendulum line
        pend_ln.set_data([btop[0], ptop[0]], [btop[1], ptop[1]])
        pend_ln.set_3d_properties([btop[2], ptop[2]])
        pend_pt.set_data([ptop[0]], [ptop[1]])
        pend_pt.set_3d_properties([ptop[2]])

        # Angle plot
        xs_plot = np.arange(i + 1)
        ln_t1.set_data(xs_plot, np.rad2deg(theta1_hist[: i + 1]))
        ln_t2.set_data(xs_plot, np.rad2deg(theta2_hist[: i + 1]))

        return (
            [trail_line, axle_pt, body_ln, pend_ln, body_pt, pend_pt, axle_bar]
            + lw_lines + rw_lines + [ln_t1, ln_t2]
        )

    print(f"  Rendering {len(states)} frames...")
    ani = animation.FuncAnimation(
        fig, update, frames=len(states), interval=30, blit=True, repeat=True
    )
    ani.save(filename, writer="pillow", fps=25, dpi=80)
    plt.close(fig)
    print(f"  Saved: {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="3D visualization of self-balancing robot")
    parser.add_argument(
        "-m", "--model", choices=["ppo", "bc", "lqr"], default="ppo",
    )
    parser.add_argument(
        "-s", "--stochastic", action="store_true",
        help="Stochastic policy (PPO only)",
    )
    parser.add_argument(
        "-n", "--steps", type=int, default=600,
    )
    parser.add_argument(
        "-o", "--output", type=str, default=None,
    )
    parser.add_argument(
        "--target-left", type=float, default=0.0,
        help="Left wheel position target (rad)",
    )
    parser.add_argument(
        "--target-right", type=float, default=0.0,
        help="Right wheel position target (rad)",
    )
    parser.add_argument(
        "--slalom", action="store_true",
        help="Straight → turn left → straight → turn right",
    )
    parser.add_argument(
        "--pi-motor", action="store_true",
        help="Enable PI velocity loop + PWM saturation motor model",
    )
    parser.add_argument(
        "--motor-gain", type=float, default=0.008,
        help="PWM → wheel accel gain (default: 0.008)",
    )
    parser.add_argument(
        "--back-emf", type=float, default=2.0,
        help="Motor back-EMF damping (default: 2.0)",
    )
    parser.add_argument(
        "--motor-tau", type=float, default=0.03,
        help="Motor 1st-order filter time constant (default: 0.03)",
    )
    parser.add_argument(
        "--pi-kp", type=float, default=25.0,
        help="PI proportional gain (default: 25, use 0 for P-only stable mode)",
    )
    parser.add_argument(
        "--pi-ki", type=float, default=35.0,
        help="PI integral gain (default: 35, reduce to <1 for stability)",
    )
    parser.add_argument(
        "--data-driven", action="store_true",
        help="Use data-driven closed-loop model (A_cl fitted from real data)",
    )
    args = parser.parse_args()

    output = args.output or f"balance_{args.model}_3d.gif"

    env_kwargs = None
    if args.data_driven:
        env_kwargs = {"data_driven": True}
    elif args.pi_motor:
        env_kwargs = {
            "use_pi_motor": True,
            "motor_gain": args.motor_gain,
            "back_emf": args.back_emf,
            "motor_tau": args.motor_tau,
            "pi_kp": args.pi_kp,
            "pi_ki": args.pi_ki,
        }

    if args.slalom:
        all_states, all_x, all_y, all_yaw = [], [], [], []
        # Position targets: equal=straight, diff=turn
        segments = [
            (200, (40.0, 40.0)),    # straight
            (200, (-15.0, 15.0)),   # turn left
            (200, (40.0, 40.0)),    # straight
            (200, (15.0, -15.0)),   # turn right
            (200, (40.0, 40.0)),    # straight
        ]
        label_base = {"ppo": "BC-reg PPO", "bc": "BC", "lqr": "LQR"}[args.model]
        label = f"{label_base} (slalom)"
        for seg_steps, drive in segments:
            s, x, y, yaw, _ = run_and_record(
                args.model, deterministic=True, steps=seg_steps, drive=drive,
                env_kwargs=env_kwargs,
            )
            all_states.extend(s)
            all_x.extend(x)
            all_y.extend(y)
            all_yaw.extend(yaw)
        make_animation_3d(all_states, all_x, all_y, all_yaw, label, output)
    else:
        states, xs, ys, yaws, label = run_and_record(
            args.model,
            deterministic=not args.stochastic,
            steps=args.steps,
            drive=(args.target_left, args.target_right),
            env_kwargs=env_kwargs,
        )
        make_animation_3d(states, xs, ys, yaws, label, output)
