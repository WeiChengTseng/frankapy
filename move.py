import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
import copy
import pdb


if __name__ == "__main__":
    fa = FrankaArm()
    
    # reset franka to its home joints
    fa.reset_joints()


    # end-effector pose control
    # print('Translation')
    T_ee_world = fa.get_pose()
    # print(T_ee_world)
    pose = copy.deepcopy(T_ee_world)
    pose.translation = [ 5.9680283e-01,  8.6971020e-05, -1.4258210e-02]
    # [ 3.16802809e-01,  8.69710215e-05, -1.41582094e-02]
    # top landmark [ 5.9680283e-01,  8.6971020e-05, -1.4258210e-02]

    fa.goto_pose(pose, duration=6)


    T_ee_rot = RigidTransform(
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(90)),
        from_frame='franka_tool', to_frame='franka_tool'
    )
    T_ee_world_target = pose * T_ee_rot
    fa.goto_pose(T_ee_world_target)

    pdb.set_trace()
    # [ 3.06802809e-01,  8.69710215e-05, -1.42582094e-02]

    # T_ee_world.translation += [0.1, 0, 0.1]
    # fa.goto_pose(T_ee_world)
    # T_ee_world.translation -= [0.1, 0, 0.1]
    # fa.goto_pose(T_ee_world)
    # print('Rotation in end-effector frame')
    # T_ee_rot = RigidTransform(
    #     rotation=RigidTransform.x_axis_rotation(np.deg2rad(45)),
    #     from_frame='franka_tool', to_frame='franka_tool'
    # )
    # T_ee_world_target = T_ee_world * T_ee_rot
    # fa.goto_pose(T_ee_world_target)
    # fa.goto_pose(T_ee_world)

    # reset franka back to home
    fa.reset_joints()