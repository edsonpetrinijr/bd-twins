"""Generates atlas.xml. Left/right are mirrored here so the XML never drifts between sides.

Run: python atlas/build.py && python atlas/check.py
Numbers and their sources: docs/atlas-sources.md.
"""

import math
from pathlib import Path

# Heights are world z with the robot standing straight. Sources in docs/atlas-sources.md.
HIP_PITCH_Z = 0.999        # pelvis origin = hip pitch axis
HIP_ROLL_DZ = 0.08         # hip yaw/roll point sits above the pitch axis (front-facing discs on the pelvis)
THIGH, SHIN, ANKLE = 0.467, 0.336, 0.196
WAIST_DZ = 0.06            # waist yaw above pelvis origin
SHOULDER_DZ = 0.514        # shoulder axis above waist -> 1.573 m (front close-up, product video)
SHOULDER_Y = 0.23          # shoulder actuator center, lateral (front close-up)
ARM_OUT = 0.0              # arm hangs straight below the shoulder actuator
HIP_Y = 0.13               # hip discs on the pelvis front (front photo says ~0.16; 0.14 keeps arms clear of thighs)
UPPER_ARM, FOREARM = 0.31, 0.28

MESHES = {}


def g(kind, cls="visual", **kw):
    attrs = " ".join(f'{k}="{v}"' for k, v in kw.items())
    return f'<geom class="{cls}" type="{kind}" {attrs}/>'


def shell(name, z0, z1, top, bot, r, material="shell", pos="0 0 0"):
    """Rounded, tapered box as a convex-hull mesh: half-sizes `bot` at z0, `top` at z1, corner radius r."""
    if name not in MESHES:
        pts = []
        for zc, (hx, hy), sgn in ((z1, top, -1), (z0, bot, 1)):
            for k in range(5):  # rounded cap
                t = math.pi / 2 * k / 4
                rr, dz = r * math.cos(t), r * (1 - math.sin(t))
                for j in range(24):  # rounded-rect outline
                    a = 2 * math.pi * j / 24
                    cx = math.copysign(hx - r, math.cos(a))
                    cy = math.copysign(hy - r, math.sin(a))
                    pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a), zc + sgn * dz))
        MESHES[name] = " ".join(f"{v:.4f}" for p in pts for v in p)
    return g("mesh", mesh=name, material=material, pos=pos)


def drum(y0, y1, r, material="frame", x=0.0, z=0.0):
    """Actuator drum along y (rounded cylinder)."""
    return g("capsule", material=material, fromto=f"{x} {y0} {z} {x} {y1} {z}", size=r)


def leg(s, side):
    # From the product renders: thigh = slim blue link; knee and ankle = blue drums (axis y) on the outside;
    # shin = dark rounded paddle on the inside, in front of the knee-ankle line; short dark foot.
    return f"""
      <body name="{side}_hip" pos="0 {s*HIP_Y} {HIP_ROLL_DZ}">
        <joint name="{side}_hip_yaw_joint" axis="0 0 1"/>
        <joint name="{side}_hip_roll_joint" axis="1 0 0" range="-0.5 0.5"/>
        <inertial pos="0 0 0" mass="0.6" diaginertia="0.002 0.002 0.002"/>
        <body name="{side}_thigh" pos="0 0 -{HIP_ROLL_DZ}">
          <joint name="{side}_hip_pitch_joint" axis="0 1 0" range="-2.2 0.6"/>
          {g("capsule", "collision", fromto=f"0 0 0 0 0 -{THIGH}", size="0.055", mass="9.0")}
          {drum(-s*0.02, s*0.06, 0.07, "frame")}
          {g("cylinder", material="rib", pos=f"0 {s*0.131} 0", euler="1.5708 0 0", size="0.04 0.004")}
          {shell("thigh_shell", -0.40, -0.06, (0.062, 0.052), (0.055, 0.048), 0.03, "shell", f"0.005 {s*0.01} 0")}
          {shell("thigh_motor", -0.40, -0.24, (0.035, 0.042), (0.035, 0.042), 0.015, "rib", f"-0.04 {s*0.01} 0")}
          <body name="{side}_shin" pos="0 0 -{THIGH}">
            <joint name="{side}_knee_joint" axis="0 1 0" range="0 2.6"/>
            {g("capsule", "collision", fromto=f"0.03 0 0 0.03 0 -{SHIN}", size="0.055", mass="4.185")}
            {drum(-s*0.0, s*0.05, 0.064, "shell")}
            {g("cylinder", material="frame", pos=f"0 {s*0.115} 0", euler="1.5708 0 0", size="0.035 0.003")}
            {shell("shin_paddle", -0.40, 0.065, (0.068, 0.028), (0.068, 0.028), 0.064, "frame", f"0.04 {-s*0.035} 0")}
            <body name="{side}_foot" pos="0 0 -{SHIN}">
              <joint name="{side}_ankle_pitch_joint" axis="0 1 0" range="-0.8 0.8"/>
              <joint name="{side}_ankle_roll_joint" axis="1 0 0" range="-0.4 0.4"/>
              {g("capsule", "collision", fromto="0 0 0 0 0 -0.116", size="0.045", mass="0.3")}
              {g("box", "collision", name=f"{side}_foot", pos="0.04 0 -0.156", size="0.16 0.06 0.04", mass="1.005")}
              {drum(-s*0.0, s*0.045, 0.058, "shell")}
              {g("cylinder", material="frame", pos=f"0 {s*0.104} 0", euler="1.5708 0 0", size="0.032 0.003")}
              {shell("ankle_block", -0.15, -0.03, (0.04, 0.035), (0.055, 0.045), 0.02, "frame")}
              {shell("sole", -0.196, -0.145, (0.15, 0.058), (0.165, 0.062), 0.02, "frame", "0.035 0 0")}
              <site name="{side}_foot" pos="0.04 0 -{ANKLE}"/>
            </body>
          </body>
        </body>
      </body>"""


def arm(s, side):
    o = s * ARM_OUT
    roll = "-0.3 3.0" if s > 0 else "-3.0 0.3"
    fingers = "\n".join(
        g("capsule", material="frame", fromto=f"{x} {o} -0.10 {x} {o + s*0.004} -0.155", size="0.011")
        for x in (-0.024, 0.0, 0.024)
    )
    return f"""
        <body name="{side}_upper_arm" pos="0 {s*SHOULDER_Y} {SHOULDER_DZ}">
          <joint name="{side}_shoulder_pitch_joint" axis="0 1 0" range="-3.1 3.1"/>
          <joint name="{side}_shoulder_roll_joint" axis="1 0 0" range="{roll}"/>
          <joint name="{side}_shoulder_yaw_joint" axis="0 0 1" range="-2.6 2.6"/>
          {g("capsule", "collision", fromto=f"0 {o} -0.05 0 {o} -{UPPER_ARM}", size="0.05", mass="2.52")}
          {g("capsule", material="frame", fromto="-0.05 0 0 0.05 0 0", size="0.082")}
          {shell("upper_arm_shell", -0.23, -0.06, (0.072, 0.066), (0.06, 0.058), 0.03, "shell", f"0 {o} 0")}
          {g("cylinder", material="rib", pos=f"0 {o} -0.25", size="0.05 0.04")}
          <body name="{side}_forearm" pos="0 {o} -{UPPER_ARM}">
            <joint name="{side}_elbow_joint" axis="0 1 0" range="-2.6 0"/>
            {g("capsule", "collision", fromto=f"0 0 0 0 0 -{FOREARM}", size="0.04", mass="1.44")}
            {drum(-s*0.05, s*0.05, 0.07, "frame")}
            {g("capsule", material="frame", fromto="0 0 -0.04 0 0 -0.24", size="0.048")}
            {shell("forearm_shell", -0.215, -0.06, (0.054, 0.054), (0.042, 0.046), 0.025, "shell", "0.01 0 0")}
            {g("cylinder", material="shell", pos="0 0 -0.262", size="0.046 0.018")}
            <body name="{side}_hand" pos="0 0 -{FOREARM}">
              <joint name="{side}_wrist_yaw_joint" axis="0 0 1" range="-2.6 2.6"/>
              <joint name="{side}_wrist_pitch_joint" axis="0 1 0" range="-1.2 1.2"/>
              <joint name="{side}_wrist_roll_joint" axis="1 0 0" range="-1.2 1.2"/>
              {g("box", "collision", pos="0 0 -0.08", size="0.04 0.025 0.08", mass="0.54")}
              {shell("palm", -0.1, 0.0, (0.042, 0.024), (0.046, 0.022), 0.012, "frame")}
              {g("box", material="shell", pos=f"0 {s*0.026} -0.055", size="0.044 0.006 0.042")}
              {fingers.replace(f" {o} ", " 0 ").replace(f" {o + s*0.004} ", f" {s*0.004} ")}
              {g("capsule", material="frame", fromto=f"0.04 {-s*0.01} -0.04 0.07 {-s*0.03} -0.09", size="0.012")}
              <site name="{side}_palm" pos="0 0 -0.08"/>
            </body>
          </body>
        </body>"""


XML = f"""<!--
  Electric Atlas (2026 product), v0.4. Built from public data only.
  GENERATED by atlas/build.py, do not edit by hand. Sources: docs/atlas-sources.md.
  Frame: x forward, y left, z up. Pelvis origin = hip pitch axis height.
  Collision geoms (group 3) carry all the mass. Visual geoms (group 2) are massless and do not collide.
  No actuators here: mjlab adds them from config.
-->
<mujoco model="atlas">
  <compiler angle="radian" autolimits="true"/>

  <asset>
    <material name="shell" rgba="0.45 0.56 0.82 1" specular="0.4" shininess="0.5"/>
    <material name="frame" rgba="0.16 0.16 0.18 1" specular="0.3" shininess="0.4"/>
    <material name="rib" rgba="0.1 0.1 0.11 1" specular="0.1"/>
    <material name="sole" rgba="0.08 0.08 0.08 1"/>
    <material name="face" rgba="0.03 0.03 0.04 1" specular="0.9" shininess="0.9"/>
    <material name="ring" rgba="1 0.92 0.72 1" emission="1"/>
@@MESHES@@
  </asset>

  <default>
    <joint damping="1" armature="0.01"/>
    <site group="5" size="0.01" rgba="1 0 0 1"/>
    <default class="collision">
      <geom group="3" rgba="0.8 0.2 0.2 0.5"/>
    </default>
    <default class="visual">
      <geom group="2" contype="0" conaffinity="0" density="0"/>
    </default>
  </default>

  <worldbody>
    <body name="pelvis" pos="0 0 {HIP_PITCH_Z}">
      <freejoint name="floating_base_joint"/>
      {g("box", "collision", name="pelvis", pos="0 0 0.05", size="0.08 0.14 0.06", mass="11.58")}
      {drum(-0.1, 0.1, 0.062, "frame")}
      {g("cylinder", material="rib", pos="0 0 0", euler="1.5708 0 0", size="0.064 0.05")}
      {drum(-0.1, 0.1, 0.062, "frame", z=0.12)}
      {g("cylinder", material="rib", pos="0 0 0.12", euler="1.5708 0 0", size="0.064 0.05")}
      <site name="imu_in_pelvis" pos="0 0 0.05"/>
{leg(1, "left")}
{leg(-1, "right")}

      <body name="torso" pos="0 0 {WAIST_DZ}">
        <joint name="waist_yaw_joint" axis="0 0 1"/>
        {g("box", "collision", name="torso", pos="0 0 0.33", size="0.13 0.17 0.27", mass="31.95")}
        {shell("waist", 0.06, 0.19, (0.09, 0.125), (0.085, 0.115), 0.03, "frame")}
        {g("box", material="rib", pos="0.088 0 0.125", size="0.004 0.09 0.045")}
        {shell("belly", 0.18, 0.41, (0.14, 0.175), (0.13, 0.16), 0.04, "shell", "0.01 0 0")}
        {g("box", material="rib", pos="0.01 0 0.415", size="0.135 0.17 0.008")}
        {shell("yoke", 0.42, 0.6, (0.125, 0.165), (0.14, 0.18), 0.075, "frame", "0.005 0 0")}
        {shell("battery", 0.14, 0.27, (0.05, 0.12), (0.055, 0.12), 0.02, "shell", "-0.16 0 0")}
        {shell("battery", 0.14, 0.27, (0.05, 0.12), (0.055, 0.12), 0.02, "shell", "-0.16 0 0.14")}
        {g("capsule", material="frame", fromto="-0.03 0.12 0.57 -0.18 0.12 0.57", size="0.013")}
        {g("capsule", material="frame", fromto="-0.03 -0.12 0.57 -0.18 -0.12 0.57", size="0.013")}
        {g("capsule", material="frame", fromto="-0.18 0.12 0.57 -0.18 0.12 0.44", size="0.013")}
        {g("capsule", material="frame", fromto="-0.18 -0.12 0.57 -0.18 -0.12 0.44", size="0.013")}
        {g("capsule", material="frame", fromto="-0.18 0.12 0.44 -0.18 -0.12 0.44", size="0.013")}
        <site name="imu_in_torso" pos="0 0 0.33"/>

        <body name="head" pos="0 0 0.52">
          <joint name="neck_yaw_joint" axis="0 0 1"/>
          {g("sphere", "collision", name="head", pos="0.03 0 0.221", size="0.1", mass="7.29")}
          {g("box", material="frame", pos="0 0 0.09", size="0.04 0.045 0.04")}
          {g("cylinder", material="shell", pos="0.01 0 0.221", euler="0 1.5708 0", size="0.1 0.12")}
          {g("cylinder", material="ring", pos="0.135 0 0.221", euler="0 1.5708 0", size="0.1 0.015")}
          {g("cylinder", material="face", pos="0.14 0 0.221", euler="0 1.5708 0", size="0.08 0.015")}
          {g("box", material="frame", pos="0.156 0 0.206", size="0.004 0.04 0.011")}
          {g("cylinder", material="ring", pos="-0.06 0 0.33", size="0.008 0.025")}
        </body>
{arm(1, "left")}
{arm(-1, "right")}
      </body>
    </body>
  </worldbody>

  <contact>
    <exclude body1="pelvis" body2="left_thigh"/>
    <exclude body1="pelvis" body2="right_thigh"/>
  </contact>
</mujoco>
"""

XML = XML.replace("@@MESHES@@", "\n".join(f'    <mesh name="{n}" vertex="{v}"/>' for n, v in MESHES.items()))
out = Path(__file__).parent / "atlas.xml"
out.write_text(XML)
print(f"wrote {out}")
