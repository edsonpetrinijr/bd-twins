"""Generates atlas.xml. Left/right are mirrored here so the XML never drifts between sides.

Run: python atlas/build.py && python atlas/check.py
Numbers and their sources: docs/atlas-sources.md.
"""

import math
from pathlib import Path

# Segment lengths (m), measured on spec-sheet photos.
HIP_Z = 0.999
THIGH, SHIN, ANKLE = 0.467, 0.336, 0.196
WAIST_Z = 0.06            # waist yaw above hip joints
SHOULDER_Z = 0.45        # shoulder joint above waist (fit to p-004 silhouette)
UPPER_ARM, FOREARM = 0.31, 0.28
HIP_Y, SHOULDER_Y = 0.09, 0.20


def g(kind, cls="visual", **kw):
    attrs = " ".join(f'{k}="{v}"' for k, v in kw.items())
    return f'<geom class="{cls}" type="{kind}" {attrs}/>'


MESHES = {}


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


def leg(s, side):
    y = s * HIP_Y
    return f"""
      <body name="{side}_thigh" pos="0 {y} 0">
        <joint name="{side}_hip_yaw_joint" axis="0 0 1"/>
        <joint name="{side}_hip_roll_joint" axis="1 0 0" range="-0.5 0.5"/>
        <joint name="{side}_hip_pitch_joint" axis="0 1 0" range="-2.2 0.6"/>
        {g("capsule", "collision", fromto=f"0 0 0 0 0 -{THIGH}", size="0.065", mass="9.0")}
        {g("cylinder", material="frame", pos="0 0 0", euler="1.5708 0 0", size="0.072 0.07")}
        {g("cylinder", material="rib", pos="0 0 0", euler="1.5708 0 0", size="0.075 0.05")}
        {shell("thigh_shell", -0.30, -0.06, (0.078, 0.066), (0.062, 0.056), 0.03)}
        {g("cylinder", material="rib", pos="0 0 -0.35", size="0.058 0.07")}
        <body name="{side}_shin" pos="0 0 -{THIGH}">
          <joint name="{side}_knee_joint" axis="0 1 0" range="0 2.6"/>
          {g("capsule", "collision", fromto=f"0 0 0 0 0 -{SHIN}", size="0.055", mass="4.185")}
          {g("cylinder", material="shell", pos=f"0 {s*0.04} 0", euler="1.5708 0 0", size="0.068 0.03")}
          {g("cylinder", material="frame", pos="0 0 0", euler="1.5708 0 0", size="0.055 0.05")}
          {shell("shin_slab", -0.33, -0.01, (0.09, 0.05), (0.075, 0.046), 0.045, "frame", "0.045 0 0")}
          {g("cylinder", material="frame", pos="0.02 0 -0.02", euler="1.5708 0 0", size="0.078 0.05")}
          {g("box", material="frame_light", pos="0.1 0 -0.17", size="0.006 0.035 0.13")}
          <body name="{side}_foot" pos="0 0 -{SHIN}">
            <joint name="{side}_ankle_pitch_joint" axis="0 1 0" range="-0.8 0.8"/>
            <joint name="{side}_ankle_roll_joint" axis="1 0 0" range="-0.4 0.4"/>
            {g("capsule", "collision", fromto="0 0 0 0 0 -0.116", size="0.045", mass="0.3")}
            {g("box", "collision", name=f"{side}_foot", pos="0.04 0 -0.156", size="0.16 0.06 0.04", mass="1.005")}
            {g("cylinder", material="shell", pos=f"0 {s*0.035} 0", euler="1.5708 0 0", size="0.065 0.025")}
            {g("cylinder", material="frame", pos="0 0 0", euler="1.5708 0 0", size="0.042 0.045")}
            {g("box", material="frame", pos="0 0 -0.08", size="0.045 0.04 0.07")}
            {shell("foot_top", -0.16, -0.11, (0.12, 0.05), (0.15, 0.056), 0.02, "frame", "0.035 0 0")}
            {shell("sole", -0.196, -0.155, (0.18, 0.062), (0.18, 0.062), 0.02, "sole", "0.035 0 0")}
            <site name="{side}_foot" pos="0.04 0 -{ANKLE}"/>
          </body>
        </body>
      </body>"""


def arm(s, side):
    y = s * SHOULDER_Y
    roll = "-0.3 3.0" if s > 0 else "-3.0 0.3"
    fingers = "\n".join(
        g("capsule", material="frame", fromto=f"{x} 0 -0.10 {x} {s*0.004} -0.155", size="0.011")
        for x in (-0.024, 0.0, 0.024)
    )
    return f"""
        <body name="{side}_upper_arm" pos="0 {y} {SHOULDER_Z}">
          <joint name="{side}_shoulder_pitch_joint" axis="0 1 0" range="-3.1 3.1"/>
          <joint name="{side}_shoulder_roll_joint" axis="1 0 0" range="{roll}"/>
          <joint name="{side}_shoulder_yaw_joint" axis="0 0 1" range="-2.6 2.6"/>
          {g("capsule", "collision", fromto=f"0 0 0 0 0 -{UPPER_ARM}", size="0.05", mass="2.52")}
          {g("cylinder", material="frame", pos=f"0 {s*0.01} 0", euler="1.5708 0 0", size="0.07 0.06")}
          {shell("upper_arm_shell", -0.2, -0.03, (0.085, 0.075), (0.068, 0.062), 0.035)}
          {g("cylinder", material="rib", pos="0 0 -0.19", size="0.05 0.05")}
          <body name="{side}_forearm" pos="0 0 -{UPPER_ARM}">
            <joint name="{side}_elbow_joint" axis="0 1 0" range="-2.6 0"/>
            {g("capsule", "collision", fromto=f"0 0 0 0 0 -{FOREARM}", size="0.04", mass="1.44")}
            {g("cylinder", material="frame", pos="0 0 0", euler="1.5708 0 0", size="0.085 0.06")}
            {g("capsule", material="frame", fromto="0 0 -0.04 0 0 -0.24", size="0.05")}
            {shell("forearm_shell", -0.215, -0.05, (0.054, 0.054), (0.042, 0.046), 0.025, "shell", "0.01 0 0")}
            {g("cylinder", material="shell", pos="0 0 -0.262", size="0.046 0.018")}
            <body name="{side}_hand" pos="0 0 -{FOREARM}">
              <joint name="{side}_wrist_yaw_joint" axis="0 0 1" range="-2.6 2.6"/>
              <joint name="{side}_wrist_pitch_joint" axis="0 1 0" range="-1.2 1.2"/>
              <joint name="{side}_wrist_roll_joint" axis="1 0 0" range="-1.2 1.2"/>
              {g("box", "collision", pos="0 0 -0.08", size="0.04 0.025 0.08", mass="0.54")}
              {shell("palm", -0.1, 0.0, (0.042, 0.024), (0.046, 0.022), 0.012, "frame")}
              {g("box", material="shell", pos=f"0 {s*0.026} -0.055", size="0.044 0.006 0.042")}
              {fingers}
              {g("capsule", material="frame", fromto=f"0.04 {-s*0.01} -0.04 0.07 {-s*0.03} -0.09", size="0.012")}
              <site name="{side}_palm" pos="0 0 -0.08"/>
            </body>
          </body>
        </body>"""


XML = f"""<!--
  Electric Atlas (2026 product), v0.2 (visual shell from spec-sheet renders). Built from public data only.
  GENERATED by atlas/build.py, do not edit by hand. Sources: docs/atlas-sources.md.
  Frame: x forward, y left, z up. Pelvis origin = hip joint height.
  Collision geoms (group 3) carry all the mass. Visual geoms (group 2) are massless and do not collide.
  No actuators here: mjlab adds them from config.
-->
<mujoco model="atlas">
  <compiler angle="radian" autolimits="true"/>

  <asset>
    <material name="shell" rgba="0.45 0.56 0.82 1" specular="0.4" shininess="0.5"/>
    <material name="frame" rgba="0.16 0.16 0.18 1" specular="0.3" shininess="0.4"/>
    <material name="frame_light" rgba="0.26 0.26 0.29 1" specular="0.3"/>
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
    <body name="pelvis" pos="0 0 {HIP_Z}">
      <freejoint name="floating_base_joint"/>
      {g("box", "collision", name="pelvis", size="0.08 0.14 0.06", mass="12.78")}
      {shell("pelvis_shell", -0.05, 0.08, (0.09, 0.12), (0.08, 0.1), 0.04, "frame")}
      {g("cylinder", material="frame", pos=f"0 {HIP_Y} 0.03", size="0.06 0.035")}
      {g("cylinder", material="frame", pos=f"0 {-HIP_Y} 0.03", size="0.06 0.035")}
      {g("cylinder", material="frame", pos="0 0.125 0", euler="1.5708 0 0", size="0.075 0.03")}
      {g("cylinder", material="frame", pos="0 -0.125 0", euler="1.5708 0 0", size="0.075 0.03")}
      <site name="imu_in_pelvis"/>
{leg(1, "left")}
{leg(-1, "right")}

      <body name="torso" pos="0 0 {WAIST_Z}">
        <joint name="waist_yaw_joint" axis="0 0 1"/>
        {g("box", "collision", name="torso", pos="0 0 0.30", size="0.12 0.14 0.25", mass="31.95")}
        {g("cylinder", material="frame", pos="0 0 0.06", size="0.1 0.06")}
        {shell("belly", 0.09, 0.46, (0.13, 0.145), (0.105, 0.12), 0.045, "shell", "0.03 0 0")}
        {shell("chest", 0.34, 0.55, (0.12, 0.15), (0.13, 0.14), 0.05, "frame", "0.0 0 0")}
        {shell("shoulder_hump", 0.44, 0.565, (0.09, 0.05), (0.1, 0.055), 0.04, "frame", "0 0.16 0")}
        {shell("shoulder_hump", 0.44, 0.565, (0.09, 0.05), (0.1, 0.055), 0.04, "frame", "0 -0.16 0")}
        {shell("battery", 0.1, 0.25, (0.05, 0.12), (0.055, 0.12), 0.02, "shell", "-0.15 0 0")}
        {shell("battery", 0.1, 0.25, (0.05, 0.12), (0.055, 0.12), 0.02, "shell", "-0.15 0 0.165")}
        {g("box", material="shell", pos="-0.191 0.11 0.29", size="0.004 0.012 0.12")}
        {g("box", material="shell", pos="-0.191 -0.11 0.29", size="0.004 0.012 0.12")}
        {g("capsule", material="frame", fromto="-0.02 0.13 0.56 -0.17 0.13 0.56", size="0.013")}
        {g("capsule", material="frame", fromto="-0.02 -0.13 0.56 -0.17 -0.13 0.56", size="0.013")}
        {g("capsule", material="frame", fromto="-0.17 0.13 0.56 -0.17 0.13 0.46", size="0.013")}
        {g("capsule", material="frame", fromto="-0.17 -0.13 0.56 -0.17 -0.13 0.46", size="0.013")}
        {g("capsule", material="frame", fromto="-0.17 0.13 0.46 -0.17 -0.13 0.46", size="0.013")}
        <site name="imu_in_torso" pos="0 0 0.30"/>

        <body name="head" pos="0 0 0.52">
          <joint name="neck_yaw_joint" axis="0 0 1"/>
          {g("sphere", "collision", name="head", pos="0 0 0.21", size="0.11", mass="7.29")}
          {g("cylinder", material="frame", pos="0 0 0.07", size="0.065 0.05")}
          {g("cylinder", material="shell", pos="0.03 0 0.2", euler="0 1.5708 0", size="0.11 0.105")}
          {g("cylinder", material="ring", pos="0.135 0 0.2", euler="0 1.5708 0", size="0.112 0.012")}
          {g("cylinder", material="face", pos="0.139 0 0.2", euler="0 1.5708 0", size="0.092 0.012")}
          {g("box", material="frame", pos="0.153 0 0.18", size="0.004 0.045 0.012")}
          {g("cylinder", material="ring", pos="-0.05 0 0.32", size="0.008 0.025")}
        </body>
{arm(1, "left")}
{arm(-1, "right")}
      </body>
    </body>
  </worldbody>
</mujoco>
"""

out = Path(__file__).parent / "atlas.xml"
XML = XML.replace("@@MESHES@@", "\n".join(f'    <mesh name="{n}" vertex="{v}"/>' for n, v in MESHES.items()))
out.write_text(XML)
print(f"wrote {out}")
