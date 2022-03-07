#!/usr/bin/env python2

import sys
import copy
from tabnanny import check
import numpy as np
from matplotlib import transforms
import rospy
import rospkg
from math import pi, sqrt
from std_msgs.msg import String
from math import degrees, radians
from tf.transformations import quaternion_from_euler, translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_matrix
from rospy import Time
from tf import TransformBroadcaster, TransformListener

from fiducial_msgs.msg import(
	FiducialTransformArray
)
from gazebo_msgs.msg import(
	ModelState

)
from gazebo_msgs.srv import (
	
	SpawnModel,
	SpawnModelResponse,
	SetModelState,
	SetModelStateResponse,
	DeleteModel,
	GetModelProperties,
	GetModelPropertiesResponse,
	GetModelState,
	GetModelStateResponse
)
from geometry_msgs.msg import (
	
	PoseStamped,
	Pose,
	Point,
	Vector3,
	Quaternion,
	Transform,
	TransformStamped
)


class Gazebo():
	
	def check_models(self, model_name):
	
		try:
			rospy.wait_for_service('/gazebo/get_model_properties')
			check_model = rospy.ServiceProxy('/gazebo/get_model_properties',GetModelProperties)
			resp_model = GetModelPropertiesResponse()
			resp_model = check_model(model_name)
			return resp_model.success
		except rospy.ServiceException, e:
			rospy.logerr("Getmodel properties service call failed: {0}".format(e))
		

	def move_model(self, model_name, model_pose, reference_frame = "base" ):

		
		state_msg = ModelState()
		state_msg.model_name = model_name	
		state_msg.pose = model_pose
		state_msg.reference_frame = reference_frame

		rospy.wait_for_service('/gazebo/set_model_state')
		model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		new_model_state = SetModelStateResponse()
		new_model_state = model_state(state_msg)

		return new_model_state.success	

	def load_gazebo_models(self, model_name, model_path, model_pose):
		# Get Models' Path
		model_dir = rospkg.RosPack().get_path('sawyer_project')+"/models/"
		# Load Table SDF
		model_xml = ''
		with open (model_dir + model_path, "r") as model_file:
			model_xml= model_file.read().replace('\n', '')

		if 'sdf' in model_path:
			rospy.wait_for_service('/gazebo/spawn_sdf_model')
			try:
				if self.check_models(model_name) is False:
					spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
					resp_sdf = SpawnModelResponse()
					resp_sdf = spawn_sdf(model_name, model_xml, "/",
										model_pose, "base" )
					print(resp_sdf.success)
					rospy.sleep(1)
				else:
					print('Model is found')
			except rospy.ServiceException, e:
				rospy.logerr("Spawn SDF service call failed: {0}".format(e))
		elif 'urdf' in model_path:
			rospy.wait_for_service('/gazebo/spawn_urdf_model')
			try:
				if self.check_models(model_name) is False:
					spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
					resp_urdf = SpawnModelResponse() 
					resp_urdf = spawn_urdf(model_name, model_xml, "/",
										model_pose, "base" )
					print(resp_urdf.success)
					rospy.sleep(1)
				else:
					print('Model is found')
			except rospy.ServiceException, e:
				rospy.logerr("Spawn URDF service call failed: {0}".format(e))

	def get_pose(self, model_name):
		try:
			get_pose_model = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
			resp_model = GetModelStateResponse()
			resp_model = get_pose_model(model_name, "base")


			return resp_model.pose
		except rospy.ServiceException, e:
			rospy.logerr("Getmodel properties service call failed: {0}".format(e))

	def add_box(self,model_name):
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()


class Camera():

	def __init__(self):
		self.gazebo = Gazebo()
		self.world_coords = Vector3(0,0,0)


	def getCamToWorld(self):
		
		cam_pose = self.gazebo.get_pose('kinect')

		transform = Transform()

		transform.translation.x = sqrt((cam_pose.position.x - self.world_coords.x)**(2))
		transform.translation.y = sqrt((cam_pose.position.y - self.world_coords.y)**(2))
		transform.translation.z = sqrt((cam_pose.position.z - self.world_coords.z)**(2))
		transform.rotation.x = cam_pose.orientation.x
		transform.rotation.y = cam_pose.orientation.y
		transform.rotation.z = cam_pose.orientation.z
		transform.rotation.w = cam_pose.orientation.w


		return transform
	
	def getCamToMarker(self, markerID):

		t = rospy.wait_for_message('/fiducial_transforms',FiducialTransformArray)
		# rospy.Subscriber('/fiducial_transforms',FiducialTransformArray)
		# print(t)
		tf = t.transforms
		# print(np.shape(tf))
		# fiducialtransform = []
		# print(tf.transform)
		for x in tf:
			if x.fiducial_id == markerID:
				tag_tform = x.transform
				print(tag_tform)
				return tag_tform

		return None
			

	def broadcastMarkerToWorld(self, markerID):

		t1 = self.getCamToWorld() 	# geometry_msg.Transform
		
		if not None:
			t2 = self.getCamToMarker(markerID)	# geometry_msg.Transform

		# print(t2)
		# print(t1)


		basetrans = translation_matrix((t1.translation.x,
										t1.translation.y,
										t1.translation.z))

		baserot = quaternion_matrix((t1.rotation.x,
									t1.rotation.y,
									t1.rotation.z,
									t1.rotation.w))

		# rospy.loginfo(tag + " basetrans: " + str(basetrans))
		# rospy.loginfo(tag +" baserot: " + str(baserot))

		tform1 = np.matmul(basetrans, baserot)

		basetrans2 = translation_matrix((t2.translation.x,
										t2.translation.y,
										t2.translation.z))

		baserot2 = quaternion_matrix((t2.rotation.x,
									t2.rotation.y,
									t2.rotation.z,
									t2.rotation.w))

		# rospy.loginfo(tag + " basetrans: " + str(basetrans))
		# rospy.loginfo(tag +" baserot: " + str(baserot))

		tform2 = np.matmul(basetrans2, baserot2)

		world_to_marker = tform1*tform2

		trans = translation_from_matrix(world_to_marker)
		quat = quaternion_from_matrix(world_to_marker)

		b = TransformBroadcaster()

		transformStamped = TransformStamped()
   
		transformStamped.header.stamp = rospy.Time.now()
		transformStamped.header.frame_id = "world"
		# transformStamped.child_frame_id = sys.argv[1]

		# translation
		transformStamped.transform.translation.x = trans[0]
		transformStamped.transform.translation.y = trans[1]
		transformStamped.transform.translation.z = trans[2]

		# quaternion 
		transformStamped.transform.rotation.x = quat[0]
		transformStamped.transform.rotation.y = quat[1]
		transformStamped.transform.rotation.z = quat[2]
		transformStamped.transform.rotation.w = quat[3]

		b.sendTransform(transformStamped.transform.translation,transformStamped.transform.rotation,'block', rospy.Time(0), '/world')
		rospy.spin()		

	
	# def listener():
		
	# 	listener = TransformListener()

	# 	listen_msg = listener.lookupTransform('/broadcaster',rospy.Time(0))

	# 	pose = geometry_msgs.msg.Pose()

	# 	pose.position.x = listen_msg.translation.x 
	# 	pose.position.y = listen_msg.translation.y 
	# 	pose.position.z = listen_msg.translation.z 

	# 	pose.orientation.x = listen_msg.rotation.x
	# 	pose.orientation.y = listen_msg.rotation.y
	# 	pose.orientation.z = listen_msg.rotation.z
	# 	pose.orientation.w = listen_msg.rotation.w

	# 	return pose 
		


def main():

	rospy.init_node('Perception', anonymous = True)
	cam = Camera()

	while not rospy.is_shutdown():
		cam.broadcastMarkerToWorld(0)



if __name__ == '__main__':
	main()
