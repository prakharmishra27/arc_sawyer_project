#!/usr/bin/env python2

import rospy
from perception import Gazebo
# from manipulation import SawyerManipulation
from geometry_msgs.msg import (
	Quaternion,
	Pose,
	Point
)

def main():
	try:
		# m = SawyerManipulation()
		obj = Gazebo()
		# obj.load_gazebo_models("abcsdf")
		obj.load_gazebo_models('cafe_table', 'cafe_table/model.sdf', model_pose=Pose(position=Point(x=1.4, y=0.0, z=-0.93)))	
		obj.load_gazebo_models('block', 'block/model.sdf', model_pose=Pose(position=Point(x=1.0, y=0.0, z=-0.157)))
		obj.load_gazebo_models('kinect', 'kinect/model.sdf', model_pose=Pose(position=Point(x=1.15, y=0.0, z=0.55), orientation=Quaternion(x=0.629,y=0,z=-0.707,w=0.0)))
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return


if __name__ == '__main__':
	main()



# # load_gazebo_models()
# 		load_gazebo_models('cafe_table','cafe_table/model.sdf', model_pose=Pose(position=Point(x=1.4, y=0.0, z=-0.93)))	
# 		load_gazebo_models('block', 'block/model.sdf', model_pose=Pose(position=Point(x=1.0, y=0.0, z=-0.157)))

		

		# pose_goal = geometry_msgs.msg.Pose()
		# print(pose_goal)
		# pose_goal.position.x = 1.0
		# pose_goal.position.y = 0.0
		# pose_goal.position.z = 0.55

		# # Pose Orientation
		# quaternion = quaternion_from_euler(radians(0),radians(90),radians(-180) )

		# pose_goal.orientation.x = quaternion[0]
		# pose_goal.orientation.y = quaternion[1]
		# pose_goal.orientation.z = quaternion[2]
		# pose_goal.orientation.w = quaternion[3]
		

		# load_gazebo_models('kinect', 'kinect/model.sdf', model_pose= pose_goal)
		# # move_model('cafe_table',model_pose=Pose(position=Point(x=1.3, y=0.0, z=-0.93)))
		# move_model('block', model_pose=Pose(position=Point(x=0.92, y=0.0, z=-0.157)), reference_frame="base")
# 		fiducial_pose()


# 		add_box('cafe_table')
# 		# add_box('block')
		
# 		m.go_to_pose_goal()
# 		rospy.sleep(2)		
# 		# rospy.sleep(3)
		
		
# 		# rospy.init_node('sawyer_basics', anonymous=True)
# 		# gripper_control()