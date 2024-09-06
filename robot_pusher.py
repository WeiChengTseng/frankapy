import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
import copy
import pdb

from frankapy.franka_constants import FrankaConstants as FC
from frankapy.skill_list import FC


class FrankArmPusher(FrankaArm):
    def __init__(self) -> None:
        super().__init__()
        pass

    def goto_pose(
        self,
        tool_pose,
        duration=3,
        use_impedance=True,
        dynamic=False,
        buffer_time=FC.DEFAULT_TERM_BUFFER_TIME,
        force_thresholds=None,
        torque_thresholds=None,
        cartesian_impedances=None,
        joint_impedances=None,
        block=True,
        ignore_errors=True,
        ignore_virtual_walls=False,
        skill_desc="GoToPose",
    ):

        return super().goto_pose(
            tool_pose,
            duration,
            use_impedance,
            dynamic,
            buffer_time,
            force_thresholds,
            torque_thresholds,
            cartesian_impedances,
            joint_impedances,
            block,
            ignore_errors,
            ignore_virtual_walls,
            skill_desc,
        )


if __name__ == "__main__":
    fa = FrankArmPusher()
    tool_pose = RigidTransform(
        translation=[0.115, 0, 0],
        # rotation=RigidTransform.z_axis_rotation(np.deg2rad(-20)),
        from_frame="franka_tool",
        to_frame="franka_tool_base",
    )
    fa.set_tool_delta_pose(tool_pose)

    # pose.translation += [0.09, 0, -0.5]
    # pose.translation += [0.12, 0.2, -0.5]
    # pose.translation = [6.780283e-01, 8.6971020e-05, 0]
    # [ 3.16802809e-01,  8.69710215e-05, -1.41582094e-02]
    # top landmark [ 5.9680283e-01,  8.6971020e-05, -1.4258210e-02]

    # reset franka to its home joints
    fa.reset_joints()

    # end-effector pose control
    T_ee_world = fa.get_pose()
    pose = copy.deepcopy(T_ee_world)



    T_ee_rot = RigidTransform(
        translation=[0.1, -0.05, 0.50],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(90)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = pose * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=7)

    pose = fa.get_pose()
    pose.translation += [0.0, -0.22, 0]
    fa.goto_pose(pose, duration=10)


    fa.reset_joints()

    # ---------

    pose = fa.get_pose()

    T_ee_rot = RigidTransform(
        # translation=[0.05, -0.05, 0.50],
        translation=[0.05, -0.05, 0.495],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(70)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = pose * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=8, ignore_virtual_walls=True)

    pose = fa.get_pose()

    pose.translation += [0.05, -0.2, -0.02]
    fa.goto_pose(pose, duration=10, ignore_virtual_walls=True)


    fa.reset_joints()

    # --------

    pose = fa.get_pose()

    T_ee_rot = RigidTransform(
        translation=[-0.03, -0.05, 0.5],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(40)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = pose * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=8, ignore_virtual_walls=True)

    pose = fa.get_pose()

    pose.translation += [0.1, -0.25, -0.04]
    fa.goto_pose(pose, duration=10, ignore_virtual_walls=True)


    fa.reset_joints()

    # ---------
    pose = fa.get_pose()

    T_ee_rot = RigidTransform(
        translation=[0.03, 0.27, 0.495],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(-90)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = pose * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=12, ignore_virtual_walls=True)

    pose = fa.get_pose()

    pose.translation += [0.0, 0.1, -0.0]
    fa.goto_pose(pose, duration=10, ignore_virtual_walls=True)


    fa.reset_joints()




