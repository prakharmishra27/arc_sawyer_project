#!/usr/bin/env python2

import sys
import copy
import moveit_commander
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
from tf import TransformBroadcaster, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import geometry_msgs.msg
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

	@staticmethod
	def check_models(model_name):
	
		try:
			rospy.wait_for_service('/gazebo/get_model_properties')
			check_model = rospy.ServiceProxy('/gazebo/get_model_properties',GetModelProperties)
			resp_model = GetModelPropertiesResponse()
			resp_model = check_model(model_name)
			return resp_model.success
		except rospy.ServiceException, e:
			rospy.logerr("Getmodel properties service call failed: {0}".format(e))
		
	
	@staticmethod
	def move_model( model_name, model_pose, reference_frame = "base" ):

		
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
					resp_sdf = spawn_sdf(model_name, model_xml, "/robot",
										model_pose, "sawyer::base")
					
					rospy.sleep(1)
					return resp_sdf.success
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
					resp_urdf = spawn_urdf(model_name, model_xml, "/robot",
										model_pose, "sawyer::base" )
				
					rospy.sleep(1)
					return	resp_urdf.success
				else:
					print('Model is found')
			except rospy.ServiceException, e:
				rospy.logerr("Spawn URDF service call failed: {0}".format(e))


	@staticmethod
	def get_pose(model_name):

		rospy.wait_for_service('/gazebo/get_model_state')
		try:
			get_pose_model = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
			resp_model = GetModelStateResponse()
			resp_model = get_pose_model(model_name, "sawyer::base")
			if resp_model.success:
				print("GetModelState successful")
				return resp_model.pose
			else:
				print("GetModelState failed")
				return None
		except rospy.ServiceException, e:
			rospy.logerr("Getmodel properties service call failed: {0}".format(e))



class Camera():

	def __init__(self):
		self.gazebo = Gazebo()
		self.world_coords = Vector3(0,0,0)
		self.b1 = TransformBroadcaster()
		self.b2 = TransformBroadcaster()
		self.c1 = TransformBroadcaster()
		self.listener = TransformListener()

	'''
	def getWorldToCam(self):
		
		cam_pose = self.gazebo.get_pose('kinect')

		print(cam_pose)

		transform =Transform()	

		transform.translation.x = sqrt((cam_pose.position.x - self.world_coords.x)**(2))
		transform.translation.y = sqrt((cam_pose.position.y - self.world_coords.y)**(2))
		transform.translation.z = sqrt((cam_pose.position.z - self.world_coords.z)**(2))
		transform.rotation.x = cam_pose.orientation.x
		transform.rotation.y = cam_pose.orientation.y
		transform.rotation.z = cam_pose.orientation.z
		transform.rotation.w = cam_pose.orientation.w


		return transform
	def broadcastCamToWorld(self):It's rare that an entire tree is static, often static frames are interspursed with dynamic frames so maintaining a separate API. If you know what you're looking for is entirely static you can always do things like setup the listener, query for the value and then shut it down, saving the resultant transform.
		cam_pose = self.gazebo.get_pose('kinect')

		# print(cam_pose)

		transform = geometry_msgs.msg.TransformStamped()

		# transform.header.stamp = rospy.Time.now()
		# transform.header.frame_id = "world"
		# transform.child_frame_id = "cam"


		transform.transform.translation.x = cam_pose.position.x 
		transform.transform.translation.y = cam_pose.position.y 
		transform.transform.translation.z = cam_pose.position.z 
		transform.transform.rotation.x = cam_pose.orientation.x
		transform.transform.rotation.y = cam_pose.orientation.y
		transform.transform.rotation.z = cam_pose.orientation.z
		transform.transform.rotation.w = cam_pose.orientation.w

		trans = (transform.transform.translation.x , transform.transform.translation.y ,transform.transform.translation.z)
		rot = (transform.transform.rotation.x , transform.transform.rotation.y ,transform.transform.rotation.z, transform.transform.rotation.w)

		# Change to StaticTransformBroadcaster
		self.c1.sendTransform(trans, rot, rospy.Time.now(),"camera_link_optical" , "world")

		# return transform
'''

	def getCamToMarker(self, markerID):
		t = rospy.wait_for_message('/fiducial_transforms',FiducialTransformArray)
		tform = t.transforms
		for x in tform:
			if x.fiducial_id == markerID:
				tag_tform = x.transform
				return tag_tform
		return None
	
	def broadcastMarkerToWorld(self, markerID):
		try:
			self.listener.waitForTransform("/world", "/camera_link", rospy.Time(0), rospy.Duration(2.0))
			(transCamToWorld, rotCamToWorld) = self.listener.lookupTransform('world', 'camera_link', rospy.Time(0)) #target, source, time
			vec1 = translation_matrix(transCamToWorld)
			quat1 = quaternion_matrix(rotCamToWorld)
			tform1 = np.dot(vec1, quat1)

			tformCamToMarker = self.getCamToMarker(markerID)
			vec2 = translation_matrix([tformCamToMarker.translation.x,tformCamToMarker.translation.y,tformCamToMarker.translation.z])
			quat2 = quaternion_matrix([tformCamToMarker.rotation.x,tformCamToMarker.rotation.y,tformCamToMarker.rotation.z,tformCamToMarker.rotation.w])

			# print(quat2)
			# quat23x3 = quat2[:3,0:3]
			tform2 = np.dot(vec2, quat2)

			# rotateByNinety = np.array([[0, 0, -1 ,0],
			# 							[0, 1, 0, 0],
			# 							[1, 0, 0,0 ],
			# 							[0, 0, 0,1 ]])

			# # print(np.shape(rotateByNinety))

			# rotatedQuat = np.dot(rotateByNinety,tform2)
			# # zeros = np.array([[0],[0],[0]])
			# # rot_con1 = np.concatenate((rotatedQuat,zeros), axis=1)
			# # zerosnones = np.array([[0,0,0,1]])
			# # rot_con2 = np.concatenate((rot_con1,zerosnones), axis=0)
			# # print(rot_con2)

			total_tform = np.dot(tform1.T, tform2.T)

			transBroadcast1 = translation_from_matrix(total_tform)
			quatBroadcast1 = quaternion_from_matrix(total_tform)

			# transBroadcast2 = translation_from_matrix(vec2)
			# quatBroadcast2 = quaternion_from_matrix(rotatedQuat)


			self.b1.sendTransform(transBroadcast1, quatBroadcast1, rospy.Time.now(), 'block1','world')
			# self.b2.sendTransform(transBroadcast2, quatBroadcast2, rospy.Time.now(), 'block','/camera_link_optical')

		except Exception as e:
			print(e)
			pass

def main():
	rospy.init_node('Perception', anonymous = True)
	print("TEST1")
	cam = Camera()	
	print("TEST2")
	while not rospy.is_shutdown():
		
		cam.broadcastMarkerToWorld(1)

if __name__ == '__main__':
	main()











# t1 = self.getWorldToCam() 	# geometry_msg.Transform

# 		# tfs = t1.transform

# 		t2 = self.getCamToMarker(markerID)	# geometry_msg.Transform

# 		# vec1 = translation_matrix([t1.translation.x,t1.translation.y,t1.translation.z])
# 		# quat1 = quaternion_matrix([t1.rotation.x,t1.rotation.y,t1.rotation.z,t1.rotation.w])
# 		# tform1 = np.dot(vec1, quat1)

# 		# vec2 = translation_matrix([t2.translation.x,t2.translation.y,t2.translation.z])
# 		# quat2 = quaternion_matrix([t2.rotation.x,t2.rotation.y,t2.rotation.z,t2.rotation.w])
# 		# tform2 = np.dot(vec2, quat2)

# 		# total_tform = np.dot(tform2, tform1.T)

# 		# translation = translation_from_matrix(total_tform)
# 		# rotation = quaternion_from_matrix(total_tform)

# 		# gives translation matrix 4x4 from world to cam
# 		basetrans = np.array(translation_matrix((t1.translation.x,
# 										t1.translation.y,
# 										t1.translation.z)))


# 		# gives just translation part of matrix 1x4
# 		trans_basetrans = translation_from_matrix(basetrans)
		
# 		# gives rotation matrix 4x4
# 		baserot = np.array(quaternion_matrix((t1.rotation.x,
# 									t1.rotation.y,
# 									t1.rotation.z,
# 									t1.rotation.w)))
		
# 		# gives quaternion from rotation matrix 1x4
# 		quat_baserot = quaternion_from_matrix(baserot)
				
# 		# extract the translational part
# 		basetrans_t = basetrans[:,3:4]
# 		# extract the rotational part
# 		baserot_r = baserot[:4,0:3]
		
# 		# clubbed the above extracted parts to form a tranform matrix 4x4
# 		tform1 = np.concatenate((baserot_r,basetrans_t), axis=1)
		
# 		# gives translation matrix 4x4 from marker to cam
# 		basetrans2 = np.array(translation_matrix((t2.translation.x,
# 										t2.translation.y,
# 										t2.translation.z)))

# 		# gives just translation part of matrix 1x4
# 		trans_basetrans2 = translation_from_matrix(basetrans2)
		
# 		# gives rotation matrix 4x4
# 		baserot2 = np.array(quaternion_matrix((t2.rotation.x,
# 									t2.rotation.y,
# 									t2.rotation.z,
# 									t2.rotation.w)))
		
# 		# gives quaternion from rotation matrix 1x4
# 		quat_baserot2 = quaternion_from_matrix(baserot2)
		

# 		# extracted the tranlsational component
# 		basetrans2_t = basetrans2[:,3:4]

# 		# extracted the rotational component
# 		baserot2_r = baserot2[:4,0:3]

# 		tform2 =  np.concatenate((baserot2_r,basetrans2_t), axis=1)
# 		# print(tform2)

# 		world_to_marker = tform1*tform2
# 		# print(world_to_marker)

# 		trans = translation_from_matrix(world_to_marker)
# 		# print(trans)
# 		quat = quaternion_from_matrix(world_to_marker)
# 		# # print(quat)

		

# 		translation1 = (trans_basetrans[0],trans_basetrans[1], trans_basetrans[2] )
# 		translation2 = (trans_basetrans2[0],trans_basetrans2[1], trans_basetrans2[2] )
# 		# print(translation)
# 		# translation = (1,1, 1)
# 		rotation1 = (quat_baserot[0],quat_baserot[1],quat_baserot[2],quat_baserot[3])
# 		rotation2 = (quat_baserot2[0],quat_baserot2[1],quat_baserot2[2],quat_baserot2[3])
	

# 		self.b1.sendTransform(translation1,rotation1, rospy.Time.now(), 'cam','/world')
# 		self.b2.sendTransform(translation2,rotation2, rospy.Time.now(), 'block','cam')
		