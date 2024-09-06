import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
import copy
import pdb

from frankapy.franka_constants import FrankaConstants as FC
from frankapy.skill_list import FC

if __name__ == "__main__":
    fa = FrankaArm()
    tool_pose = RigidTransform(
        translation=[0.112, 0, 0],
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
    T_ee_world = fa.get_pose()

    T_ee_rot = RigidTransform(
        translation=[0.1, 0, 0.50],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = T_ee_world * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=7)


    T_ee_world = fa.get_pose()
    T_ee_rot = RigidTransform(
        # translation=[0.1, -0.05, 0.50],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(90)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = T_ee_world * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=7)


    T_ee_world = fa.get_pose()
    T_ee_rot = RigidTransform(
        # translation=[0.1, -0.05, 0.50],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(-90)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = T_ee_world * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=7)
    
    T_ee_world = fa.get_pose()
    T_ee_rot = RigidTransform(
        # translation=[0.1, -0.05, 0.50],
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(-90)),
        from_frame="franka_tool",
        to_frame="franka_tool",
    )
    T_ee_world_target = T_ee_world * T_ee_rot
    fa.goto_pose(T_ee_world_target, duration=7)

    fa.reset_joints()