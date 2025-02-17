# motion_planning.py
# Motion Planning using MoveIt for UR-3
import moveit_commander
import rospy

def move_to_position(target_pose):
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_pose_target(target_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    return plan
