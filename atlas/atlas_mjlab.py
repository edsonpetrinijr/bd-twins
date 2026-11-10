"""Atlas in mjlab: robot config (actuators, collisions, keyframe) + velocity-tracking task.

Importing this module registers:
  Mjlab-Velocity-Flat-BD-Atlas
  Mjlab-Velocity-Rough-BD-Atlas

Train:  python atlas/train.py Mjlab-Velocity-Flat-BD-Atlas --env.scene.num-envs 4096
Actuator numbers are estimates; derivation in docs/atlas-sources.md ("Actuators").
"""

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RayCastSensorCfg, TerrainHeightSensorCfg
from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.utils.spec_config import CollisionCfg

ATLAS_XML = Path(__file__).parent / "atlas.xml"

##
# Actuators. (effort Nm, velocity rad/s, reflected inertia kg m^2), grouped by joint.
##

GROUPS = {
    "hip_knee": ((".*_hip_pitch_joint", ".*_knee_joint"), 400.0, 12.0, 0.10),
    "hip_roll": ((".*_hip_roll_joint",), 300.0, 12.0, 0.10),
    "hip_yaw": ((".*_hip_yaw_joint",), 150.0, 12.0, 0.05),
    "ankle_pitch": ((".*_ankle_pitch_joint",), 250.0, 12.0, 0.05),
    "ankle_roll": ((".*_ankle_roll_joint",), 120.0, 12.0, 0.03),
    "waist": (("waist_yaw_joint",), 200.0, 10.0, 0.08),
    "shoulder": ((".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"), 200.0, 12.0, 0.05),
    "shoulder_yaw": ((".*_shoulder_yaw_joint",), 100.0, 15.0, 0.02),
    "elbow": ((".*_elbow_joint",), 120.0, 15.0, 0.02),
    "wrist": ((".*_wrist_.*_joint",), 30.0, 20.0, 0.005),
    "neck": (("neck_yaw_joint",), 10.0, 10.0, 0.002),
}

# PD gains from reflected inertia: 10 Hz natural frequency, damping ratio 2 (mjlab convention).
OMEGA, ZETA = 2 * 3.14159265 * 10.0, 2.0

ACTUATORS = tuple(
    BuiltinPositionActuatorCfg(
        target_names_expr=names,
        stiffness=arm * OMEGA**2,
        damping=2 * ZETA * arm * OMEGA,
        effort_limit=effort,
        armature=arm,
    )
    for names, effort, _vel, arm in GROUPS.values()
)

ACTION_SCALE = {n: 0.25 * a.effort_limit / a.stiffness for a in ACTUATORS for n in a.target_names_expr}

##
# Robot.
##

STAND = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.975),  # pelvis origin = hip pitch axis, knees slightly bent
    joint_pos={
        ".*_hip_pitch_joint": -0.25,
        ".*_knee_joint": 0.5,
        ".*_ankle_pitch_joint": -0.25,
        ".*_elbow_joint": -0.35,
        "left_shoulder_roll_joint": 0.08,
        "right_shoulder_roll_joint": -0.08,
    },
    joint_vel={".*": 0.0},
)

FEET = r"^(left|right)_foot_collision$"
COLLISIONS = CollisionCfg(
    geom_names_expr=(".*_collision",),
    contype=1,
    conaffinity=1,
    condim={FEET: 3, ".*_collision": 1},
    priority={FEET: 1, ".*": 0},
    friction={FEET: (0.8,)},
)


def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(ATLAS_XML))


def atlas_robot_cfg() -> EntityCfg:
    return EntityCfg(
        init_state=STAND,
        collisions=(COLLISIONS,),
        spec_fn=get_spec,
        articulation=EntityArticulationInfoCfg(actuators=ACTUATORS, soft_joint_pos_limit_factor=0.9),
    )


##
# Velocity task.
##


def atlas_velocity_env_cfg(play: bool = False, flat: bool = True) -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()
    cfg.scene.entities = {"robot": atlas_robot_cfg()}
    feet_sites = ("left_foot", "right_foot")

    for sensor in cfg.scene.sensors or ():
        if sensor.name == "terrain_scan":
            assert isinstance(sensor, RayCastSensorCfg) and isinstance(sensor.frame, ObjRef)
            sensor.frame.name = "pelvis"
        if sensor.name == "foot_height_scan":
            assert isinstance(sensor, TerrainHeightSensorCfg)
            sensor.frame = tuple(ObjRef(type="site", name=s, entity="robot") for s in feet_sites)

    feet_contact = ContactSensorCfg(
        name="feet_ground_contact",
        primary=ContactMatch(mode="subtree", pattern=r"^(left_foot|right_foot)$", entity="robot"),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found", "force"),
        reduce="netforce",
        num_slots=1,
        track_air_time=True,
    )
    self_contact = ContactSensorCfg(
        name="self_collision",
        primary=ContactMatch(mode="subtree", pattern="pelvis", entity="robot"),
        secondary=ContactMatch(mode="subtree", pattern="pelvis", entity="robot"),
        fields=("found", "force"),
        reduce="none",
        num_slots=1,
        history_length=4,
    )
    cfg.scene.sensors = (cfg.scene.sensors or ()) + (feet_contact, self_contact)

    action = cfg.actions["joint_pos"]
    assert isinstance(action, JointPositionActionCfg)
    action.scale = ACTION_SCALE

    cfg.viewer.body_name = "torso"
    cfg.commands["twist"].viz.z_offset = 1.3
    cfg.events["foot_friction"].params["asset_cfg"].geom_names = ("left_foot_collision", "right_foot_collision")
    cfg.events["base_com"].params["asset_cfg"].body_names = ("torso",)

    cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
    walk = {r".*hip_pitch.*": 0.3, r".*hip_roll.*": 0.15, r".*hip_yaw.*": 0.15, r".*knee.*": 0.35,
            r".*ankle_pitch.*": 0.25, r".*ankle_roll.*": 0.1, r".*waist.*": 0.2, r".*neck.*": 0.3,
            r".*shoulder_pitch.*": 0.15, r".*shoulder_roll.*": 0.15, r".*shoulder_yaw.*": 0.1,
            r".*elbow.*": 0.15, r".*wrist.*": 0.3}
    cfg.rewards["pose"].params["std_walking"] = walk
    cfg.rewards["pose"].params["std_running"] = {k: v * 1.5 for k, v in walk.items()}
    cfg.rewards["upright"].params["asset_cfg"].body_names = ("torso",)
    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso",)
    for name in ("foot_clearance", "foot_slip"):
        cfg.rewards[name].params["asset_cfg"].site_names = feet_sites
    cfg.rewards["self_collisions"] = RewardTermCfg(
        func=mdp.self_collision_cost, weight=-1.0, params={"sensor_name": "self_collision", "force_threshold": 10.0}
    )

    if flat:
        assert cfg.scene.terrain is not None
        cfg.scene.terrain.terrain_type = "plane"
        cfg.scene.terrain.terrain_generator = None
        cfg.curriculum.pop("terrain_levels", None)
    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
    return cfg


def atlas_ppo_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True,
                            distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0,
                                              "std_type": "scalar"}),
        critic=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2,
                                       entropy_coef=0.01, num_learning_epochs=5, num_mini_batches=4,
                                       learning_rate=1.0e-3, schedule="adaptive", gamma=0.99, lam=0.95,
                                       desired_kl=0.01, max_grad_norm=1.0),
        experiment_name="atlas_velocity",
        save_interval=50,
        num_steps_per_env=24,
        max_iterations=30_000,
    )


for flat, label in ((True, "Flat"), (False, "Rough")):
    register_mjlab_task(
        task_id=f"Mjlab-Velocity-{label}-BD-Atlas",
        env_cfg=atlas_velocity_env_cfg(flat=flat),
        play_env_cfg=atlas_velocity_env_cfg(play=True, flat=flat),
        rl_cfg=atlas_ppo_cfg(),
        runner_cls=VelocityOnPolicyRunner,
    )
