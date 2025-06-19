# -*- coding: utf-8 -*-
import argparse
import time

from utils.bump_generator import generate_elliptical_cylinder_stl
from utils.update_mjcf import insert_stl_to_mjcf
from utils.lidar import lidar_observation  # 🔥 LiDAR 유틸 추가

import mujoco
import mujoco.viewer
import numpy as np


def run_simulation(xml_path="models/generated_scene.xml", torque=100.0, sim_time=10.0):
    """Run mujoco simulation for the given xml scene."""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.1]
        viewer.cam.distance = 1.5
        end_time = data.time + sim_time

        while viewer.is_running() and data.time < end_time:
            # Torque control to wheels
            data.ctrl[model.actuator("fl_motor").id] = torque
            data.ctrl[model.actuator("fr_motor").id] = torque
            data.ctrl[model.actuator("rl_motor").id] = torque
            data.ctrl[model.actuator("rr_motor").id] = torque

            # Physics step
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

            # Sensor data
            acc_z = data.sensordata[model.sensor("accel_z").adr]
            gyro_pitch = data.sensordata[model.sensor("gyro_pitch").adr]

            # LiDAR readings (좌우 32개씩)
            lidar_vec = lidar_observation(
                model,
                data,
                site_names=["lidar_left", "lidar_right"],
                num_rays=32,
                max_distance=5.0,
            )

            # print(
            #     f"time={data.time:.2f}s | acc_z={acc_z.item():.3f} m/s² | "
            #     f"pitch_rate={gyro_pitch.item():.3f} rad/s | "
            #     f"LiDAR[min]={lidar_vec.min():.2f} | LiDAR[max]={lidar_vec.max():.2f}"
            # )
            # (매 step에서)
            print(f"LiDAR vector: {np.round(lidar_vec, 2)}")
            print(f"min={lidar_vec.min():.2f}, max={lidar_vec.max():.2f}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate bumps, update scene and run simulation")
    parser.add_argument(
        "-n", "--num-bumps", type=int, default=5,
        help="number of speed bumps to place")
    parser.add_argument(
        "--seed", type=int, default=None,
        help="random seed for bump placement")
    parser.add_argument(
        "--torque", type=float, default=100.0,
        help="motor torque applied to each wheel")
    parser.add_argument(
        "--time", type=float, default=10.0,
        help="simulation duration in seconds")
    parser.add_argument(
        "--skip-generation", action="store_true",
        help="skip creating new bump STL files")
    args = parser.parse_args()

    if not args.skip_generation:
        for _ in range(args.num_bumps):
            generate_elliptical_cylinder_stl()

    insert_stl_to_mjcf(n_bump=args.num_bumps, random_seed=args.seed)
    run_simulation(torque=args.torque, sim_time=args.time)


if __name__ == "__main__":
    main()
