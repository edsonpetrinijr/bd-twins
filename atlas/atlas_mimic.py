"""Atlas motion imitation (BeyondMimic-style tracking in mjlab) on human mocap retargeted to Atlas.

Importing this module registers:
  Mjlab-Tracking-Flat-BD-Atlas   (motion: mocap/atlas_69_70.npz, walk up, squat, pick up, carry, set down)

Motion files come from mocap/retarget.py (CMU mocap, free for commercial and research use). Pick another
clip with --env.commands.motion.motion-file mocap/atlas_02_01.npz.

Train: python atlas/train.py Mjlab-Tracking-Flat-BD-Atlas --env.scene.num-envs 4096
"""

from pathlib import Path

from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.tasks.tracking.rl import MotionTrackingOnPolicyRunner
from mjlab.tasks.tracking.tracking_env_cfg import make_tracking_env_cfg

from atlas_mjlab import ACTION_SCALE, atlas_robot_cfg

MOTION = Path(__file__).resolve().parent.parent / "mocap" / "atlas_69_70.npz"


def atlas_mimic_env_cfg(play: bool = False):
    cfg = make_tracking_env_cfg()
    cfg.scene.entities = {"robot": atlas_robot_cfg()}
    cfg.scene.sensors = (ContactSensorCfg(
        name="self_collision",
        primary=ContactMatch(mode="subtree", pattern="pelvis", entity="robot"),
        secondary=ContactMatch(mode="subtree", pattern="pelvis", entity="robot"),
        fields=("found", "force"), reduce="none", num_slots=1, history_length=4),)

    cfg.actions["joint_pos"].scale = ACTION_SCALE

    motion = cfg.commands["motion"]
    assert isinstance(motion, MotionCommandCfg)
    motion.motion_file = str(MOTION)
    motion.anchor_body_name = "torso"
    motion.body_names = ("pelvis", "left_thigh", "left_shin", "left_foot", "right_thigh", "right_shin",
                         "right_foot", "torso", "left_upper_arm", "left_forearm", "left_hand",
                         "right_upper_arm", "right_forearm", "right_hand")

    cfg.events["foot_friction"].params["asset_cfg"].geom_names = ("left_foot_collision", "right_foot_collision")
    cfg.events["base_com"].params["asset_cfg"].body_names = ("torso",)
    cfg.terminations["ee_body_pos"].params["body_names"] = ("left_foot", "right_foot", "left_hand", "right_hand")
    cfg.viewer.body_name = "torso"

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        motion.pose_range = {}
        motion.velocity_range = {}
        motion.sampling_mode = "start"
    return cfg


def atlas_mimic_ppo_cfg():
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True,
                            distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0,
                                              "std_type": "scalar"}),
        critic=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2,
                                       entropy_coef=0.005, num_learning_epochs=5, num_mini_batches=4,
                                       learning_rate=1.0e-3, schedule="adaptive", gamma=0.99, lam=0.95,
                                       desired_kl=0.01, max_grad_norm=1.0),
        experiment_name="atlas_mimic",
        save_interval=250,
        num_steps_per_env=24,
        max_iterations=5_000,  # one ~9 s clip; G1's 30k default is for long multi-clip runs
    )


register_mjlab_task(
    task_id="Mjlab-Tracking-Flat-BD-Atlas",
    env_cfg=atlas_mimic_env_cfg(),
    play_env_cfg=atlas_mimic_env_cfg(play=True),
    rl_cfg=atlas_mimic_ppo_cfg(),
    runner_cls=MotionTrackingOnPolicyRunner,
)

if __name__ == "__main__":  # self-check on CPU: env builds, motion loads, a few steps stay finite
    import torch
    from mjlab.envs import ManagerBasedRlEnv

    c = atlas_mimic_env_cfg()
    c.scene.num_envs = 4
    env = ManagerBasedRlEnv(c, device="cpu")
    obs, _ = env.reset()
    print("action dim", env.action_manager.total_action_dim, {k: tuple(v.shape) for k, v in obs.items()})
    for _ in range(50):
        obs, rew, *_ = env.step(torch.zeros(4, env.action_manager.total_action_dim))
    assert torch.isfinite(rew).all()
    print("reward", rew.tolist(), "ok")
