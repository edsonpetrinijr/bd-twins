"""Studio stage for renders: floor, 3-point lights, soft ambient. Shared by render/views scripts."""

import mujoco


def stage(spec: mujoco.MjSpec, floor=(0.32, 0.33, 0.35, 1)) -> mujoco.MjSpec:
    v = spec.visual
    v.global_.offwidth, v.global_.offheight = 1600, 1600
    v.headlight.ambient = [0.25, 0.25, 0.27]
    v.headlight.diffuse = [0.35, 0.35, 0.37]
    v.headlight.specular = [0.1, 0.1, 0.1]
    v.quality.shadowsize = 8192
    v.map.shadowclip = 1.2  # tighter shadow frustum: less acne on the shells
    v.map.znear = 0.02
    w = spec.worldbody
    w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[6, 6, 0.1], rgba=list(floor))
    w.add_light(pos=[2.5, -2.0, 3.5], dir=[-0.55, 0.45, -0.7], diffuse=[0.75, 0.75, 0.78], castshadow=True)   # key
    w.add_light(pos=[-1.5, 2.5, 2.5], dir=[0.4, -0.6, -0.5], diffuse=[0.35, 0.37, 0.42], castshadow=False)  # fill
    w.add_light(pos=[-3.0, -0.5, 2.2], dir=[0.9, 0.1, -0.3], diffuse=[0.45, 0.45, 0.5], castshadow=False)  # rim
    return spec
