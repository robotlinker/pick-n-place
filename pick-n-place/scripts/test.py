import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

rospy.init_node('test', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm_group")

pose_target = group.get_current_pose()
print pose_target
pose_target.pose.position.z -= 0.3
group.set_pose_target(pose_target)
group.go()

