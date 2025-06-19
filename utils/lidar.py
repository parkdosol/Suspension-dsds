# -*- coding: utf-8 -*-
"""LiDAR utility functions for casting rays in MuJoCo."""

from __future__ import annotations
import numpy as np

try:
    import mujoco
except Exception:
    mujoco = None


def _ensure_mujoco_available() -> None:
    if mujoco is None:
        raise RuntimeError("mujoco package is required for lidar utilities")


def cast_rays_from_site(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    site_name: str,
    *,
    num_rays: int = 64,
    max_distance: float = 50.0,
    geomgroup: int = -1,
    include_static: bool = True,
    include_dynamic: bool = True,
) -> np.ndarray:
    """Cast rays in the horizontal plane from a given site."""
    _ensure_mujoco_available()

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise ValueError(f"unknown site: {site_name}")

    origin = data.site_xpos[site_id].copy().reshape(3, 1)
    mat = data.site_xmat[site_id].reshape(3, 3)

    if geomgroup == -1:
        geomgroup_array = np.ones((6, 1), dtype=np.uint8)
    else:
        geomgroup_array = np.zeros((6, 1), dtype=np.uint8)
        geomgroup_array[geomgroup, 0] = 1

    for i in range(num_rays):
        angle = 2.0 * np.pi * i / num_rays
        local_dir = np.array([np.cos(angle), np.sin(angle), 0.0])
        world_dir = mat @ local_dir
        vec = (world_dir * max_distance).reshape(3, 1)

        dist = np.asarray(max_distance, dtype=float)
        geomid = np.asarray(-1, dtype=np.int32)

        # call whichever ray API is available
        if hasattr(mujoco, "mj_ray"):
            # Newer MuJoCo API (distance returned as the function result)
            dist_val = mujoco.mj_ray(
                model,
                data,
                origin,
                vec,
                geomgroup=geomgroup,
                flg_static=int(include_static),
                bodyexclude=-1,
                geomid=geomid,
            )
            if geomid >= 0:
                distances[i] = float(dist_val)
        elif hasattr(mujoco, "mj_raycast"):
            # Older API name where distance is an output argument
            mujoco.mj_raycast(
                model,
                data,
                origin,
                vec,
                geomgroup,
                int(include_static),
                int(include_dynamic),
                dist,
                geomid,
            )
            if geomid >= 0:
                distances[i] = float(dist)
        else:
            raise RuntimeError("No MuJoCo ray casting function found")

    return distances


def lidar_observation(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    site_names: list[str],
    *,
    num_rays: int = 64,
    max_distance: float = 50.0,
) -> np.ndarray:
    """Return concatenated LiDAR observations for multiple sites."""
    readings = [
        cast_rays_from_site(
            model,
            data,
            name,
            num_rays=num_rays,
            max_distance=max_distance,
        )
        for name in site_names
    ]
    return np.concatenate(readings)
