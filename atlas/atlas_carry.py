"""Atlas carry task: the policy walks with the legs while the upper body is commanded from outside.

Importing this module registers:
  Mjlab-Velocity-Carry-BD-Atlas

Split (same idea as HOMIE / AMO-style loco-manipulation):
- Policy: 12 leg joints, tracks the twist command (as in the Flat task).
- Upper body: not in the action. An "upper" command drives the waist yaw (turns toward a random goal at
  up to 2.5 rad/s, so the torso swings like at a pick/place) and the arms between a hanging pose and the
  carry pose. The policy sees this command and must keep balance under it.
- A box is rigidly attached in front of the torso; its mass is randomized per episode (0.1-20 kg), so
  the legs learn to carry a load that turns with the torso.
In the fleet, the fleet manager sets the same command (waist angle + carry flag) and the policy walks.

Train: python atlas/train.py Mjlab-Velocity-Carry-BD-Atlas --env.scene.num-envs 4096
"""

import math
from dataclasses import dataclass

import mujoco
import torch

from mjlab.entity import Entity
from mjlab.envs.mdp import dr
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from atlas_mjlab import atlas_ppo_cfg, atlas_robot_cfg, atlas_velocity_env_cfg, get_spec

LEGS = (".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint")
BOX_POS = (0.42, 0.0, 0.0)          # in the torso frame: in front of the chest, where the fleet holds it
BOX_SIZE = (0.15, 0.175, 0.125)     # half sizes, the fleet's 0.35 x 0.30 x 0.25 m case (x = depth)
CARRY = {"shoulder_pitch": -0.55, "shoulder_roll": 0.05, "shoulder_yaw": -0.25, "elbow": -1.35}
HANG = {"shoulder_pitch": 0.0, "shoulder_roll": 0.08, "shoulder_yaw": 0.0, "elbow": -0.35}
WAIST_MAX, WAIST_RATE = 2.6, 2.5    # rad, rad/s (the fleet demo uses the same rate)


def carry_spec() -> mujoco.MjSpec:
    spec = get_spec()
    box = spec.body("torso").add_body(name="carry_box", pos=BOX_POS)
    box.mass, box.inertia, box.explicitinertial = 8.0, [0.1, 0.1, 0.1], True  # mass randomized per episode
    box.add_geom(name="carry_box_visual", type=mujoco.mjtGeom.mjGEOM_BOX, size=BOX_SIZE, contype=0,
                 conaffinity=0, group=2, mass=0, rgba=[0.72, 0.55, 0.36, 1])
    return spec


def wrap(a):
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi


@dataclass(kw_only=True)
class UpperBodyCommandCfg(CommandTermCfg):
    entity_name: str = "robot"
    zero_waist_prob: float = 0.3   # many picks happen straight ahead
    carry_prob: float = 0.7
    native: bool = False           # v2: torso heading in the world frame, full turn, rate slows with load
    slow_rate: float = 1.2         # v2: waist rate (rad/s) with the heaviest box

    def build(self, env):
        return UpperBodyCommand(self, env)


class UpperBodyCommand(CommandTerm):
    """Command = (waist angle now, waist goal, carry flag). Writes the upper-body joint targets itself."""

    cfg: UpperBodyCommandCfg

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot: Entity = env.scene[cfg.entity_name]
        n, dev = self.num_envs, self.device
        self.waist = torch.zeros(n, device=dev)
        self.goal = torch.zeros(n, device=dev)
        self.carry = torch.zeros(n, device=dev)
        self.dt = env.step_dt
        names = self.robot.joint_names
        self.waist_id = names.index("waist_yaw_joint")
        self.neck_id = names.index("neck_yaw_joint")
        self.arm_ids, carry, hang = [], [], []
        for side, s in (("left", 1.0), ("right", -1.0)):
            for k in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"):
                self.arm_ids.append(names.index(f"{side}_{k}_joint"))
                sign = s if k in ("shoulder_roll", "shoulder_yaw") else 1.0
                carry.append(CARRY[k] * sign)
                hang.append(HANG[k] * sign)
        self.carry_pose = torch.tensor(carry, device=dev)
        self.hang_pose = torch.tensor(hang, device=dev)
        self.metrics["error_waist"] = torch.zeros(n, device=dev)
        self.box_id = self.robot.indexing.body_ids[self.robot.body_names.index("carry_box")]
        self.goal_w = torch.zeros(n, device=dev)  # v2: torso heading goal in the world

    @property
    def command(self) -> torch.Tensor:
        waist = wrap(self.waist) if self.cfg.native else self.waist
        return torch.stack([waist, self.goal, self.carry], dim=-1)

    def _update_metrics(self):
        q = self.robot.data.joint_pos[:, self.waist_id]
        self.metrics["error_waist"] += torch.abs(wrap(q - self.waist)) * self.dt / self._env.max_episode_length_s

    def _resample_command(self, env_ids):
        k = len(env_ids)
        lim = math.pi if self.cfg.native else WAIST_MAX
        goal = torch.empty(k, device=self.device).uniform_(-lim, lim)
        goal[torch.rand(k, device=self.device) < self.cfg.zero_waist_prob] = 0.0
        self.goal[env_ids] = goal
        # v2: the goal is a torso heading in the world (face the pallet / belt while the legs go elsewhere)
        self.goal_w[env_ids] = self.robot.data.heading_w[env_ids] + goal
        self.carry[env_ids] = (torch.rand(k, device=self.device) < self.cfg.carry_prob).float()

    def _update_command(self, env_ids):
        if env_ids is not None:  # reset: start from the upright default
            self.waist[env_ids] = 0.0
        if self.cfg.native:
            heading = self.robot.data.heading_w
            if env_ids is not None:
                self.goal_w[env_ids] = heading[env_ids] + self.goal[env_ids]
            self.goal = wrap(self.goal_w - heading)  # waist angle that makes the torso face goal_w
            mass = self._env.sim.model.body_mass[:, self.box_id]
            rate = WAIST_RATE - (WAIST_RATE - self.cfg.slow_rate) * torch.clamp(mass / 20.0, 0.0, 1.0)
            # shortest way round; the waist is continuous, so the joint target may leave [-pi, pi]
            self.waist += torch.clamp(wrap(self.goal - self.waist), -rate * self.dt, rate * self.dt)
        else:
            step = WAIST_RATE * self.dt
            self.waist += torch.clamp(self.goal - self.waist, -step, step)
        c = self.carry.unsqueeze(-1)
        arms = c * self.carry_pose + (1 - c) * self.hang_pose
        self.robot.set_joint_position_target(self.waist.unsqueeze(-1), joint_ids=[self.waist_id])
        self.robot.set_joint_position_target(torch.zeros_like(self.waist).unsqueeze(-1), joint_ids=[self.neck_id])
        self.robot.set_joint_position_target(arms, joint_ids=self.arm_ids)


def atlas_carry_env_cfg(play: bool = False, native: bool = False):
    cfg = atlas_velocity_env_cfg(play=play, flat=True)
    robot = atlas_robot_cfg()
    robot.spec_fn = carry_spec
    cfg.scene.entities = {"robot": robot}

    action = cfg.actions["joint_pos"]
    action.actuator_names = LEGS
    action.scale = {k: v for k, v in action.scale.items() if any(s in k for s in ("hip", "knee", "ankle"))}
    cfg.commands["upper"] = UpperBodyCommandCfg(resampling_time_range=(1.5, 4.0), native=native)
    upper_obs = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "upper"})
    cfg.observations["actor"].terms["upper_command"] = upper_obs
    cfg.observations["critic"].terms["upper_command"] = upper_obs

    # Posture reward only on what the policy controls; the upper body follows the command.
    cfg.rewards["pose"].params["asset_cfg"] = SceneEntityCfg("robot", joint_names=LEGS)
    for key in ("std_standing", "std_walking", "std_running"):
        stds = cfg.rewards["pose"].params[key]
        cfg.rewards["pose"].params[key] = {k: v for k, v in stds.items()
                                           if any(s in k for s in ("hip", "knee", "ankle"))} or {".*": 0.05}

    cfg.events["box_mass"] = EventTermCfg(
        mode="reset", func=dr.body_mass,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=("carry_box",)), "operation": "abs",
                "ranges": (0.1, 20.0)})

    if native:
        # Gait: v1 shuffled (feet ~4 cm up, air_time weight 0). Reward real swing phases, and keep the
        # pelvis (not the torso, which turns on purpose) from wobbling.
        air = cfg.rewards["air_time"]
        air.weight = 0.5
        air.params.update(threshold_min=0.2, threshold_max=0.6, command_threshold=0.1)
        cfg.rewards["body_ang_vel"].weight = -0.05
        cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("pelvis",)
        # Atlas-only skill: carry "backwards" (legs walk one way, torso faces the other), so ask for more
        # backward and sideways walking than the default.
        r = cfg.commands["twist"].ranges
        r.lin_vel_x, r.lin_vel_y = (-1.0, 1.0), (-0.6, 0.6)
    return cfg


def atlas_carry_ppo_cfg():
    cfg = atlas_ppo_cfg()
    cfg.experiment_name = "atlas_carry"
    return cfg


register_mjlab_task(
    task_id="Mjlab-Velocity-Carry-BD-Atlas",
    env_cfg=atlas_carry_env_cfg(),
    play_env_cfg=atlas_carry_env_cfg(play=True),
    rl_cfg=atlas_carry_ppo_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)


def atlas_native_ppo_cfg():
    cfg = atlas_carry_ppo_cfg()  # same experiment folder, so it can resume from the v1 carry run
    cfg.max_iterations = 1_500
    return cfg


# v2 ("native"): same observation/action sizes as v1, so it fine-tunes from the v1 checkpoint:
#   python atlas/train.py Mjlab-Velocity-Native-BD-Atlas --env.scene.num-envs 4096 \
#     --agent.resume True --agent.load-run <v1 run folder> --agent.load-checkpoint model_2999.pt
register_mjlab_task(
    task_id="Mjlab-Velocity-Native-BD-Atlas",
    env_cfg=atlas_carry_env_cfg(native=True),
    play_env_cfg=atlas_carry_env_cfg(play=True, native=True),
    rl_cfg=atlas_native_ppo_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)

if __name__ == "__main__":  # self-check: build the env on CPU, step it with zero actions, print shapes
    from mjlab.envs import ManagerBasedRlEnv

    env_cfg = atlas_carry_env_cfg(native=True)
    env_cfg.scene.num_envs = 4
    env = ManagerBasedRlEnv(env_cfg, device="cpu")
    obs, _ = env.reset()
    print("action dim", env.action_manager.total_action_dim, "obs", {k: tuple(v.shape) for k, v in obs.items()})
    for _ in range(100):
        obs, rew, term, trunc, _ = env.step(torch.zeros(4, env.action_manager.total_action_dim))
    up = env.command_manager.get_command("upper")
    r = env.scene["robot"]
    print("upper cmd", up[0].tolist(), "waist q", r.data.joint_pos[0, r.joint_names.index("waist_yaw_joint")].item())
    print("box mass", env.sim.model.body_mass[:, r.indexing.body_ids[r.body_names.index("carry_box")]].tolist())
    assert env.action_manager.total_action_dim == 12
    assert torch.isfinite(rew).all()
    print("ok")
