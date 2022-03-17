#!/usr/bin/env python2

from ast import And
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Point
import perception


class Rviz():

    def __init__(self):

        rospy.init_node('Rviz', anonymous = True)  

        self.scene = moveit_commander.PlanningSceneInterface()             
       

    def add_box(self, model_name,pose_model ,model_size=(1,1,1)):
      

        clean_pose = PoseStamped()
        clean_pose.header.frame_id = "base"
        # clean_pose.header.time = rospy.Time.now()
        clean_pose.pose.position.x = pose_model.position.x
        clean_pose.pose.position.y = pose_model.position.y
        clean_pose.pose.position.z = pose_model.position.z
        
        clean_pose.pose.orientation.x = pose_model.orientation.x
        clean_pose.pose.orientation.y = pose_model.orientation.y
        clean_pose.pose.orientation.z = pose_model.orientation.z
        clean_pose.pose.orientation.w = pose_model.orientation.w
        
        # print(pose_model.header)
        # print(clean_pose.header)
        
        self.scene.add_box(model_name, clean_pose, model_size)


def main():
    rv = Rviz()

    pose_table = None
    pose_block0 = None
    pose_block1 = None
    pose_kinect = None    


    for x in range(5):
        pose_list = []
        
        try:
            pose_table = perception.Gazebo.get_pose('cafe_table')
            pose_list.append(pose_table == None)
            pose_block0 = perception.Gazebo.get_pose('block0')
            pose_list.append(pose_block0 == None)
            pose_block1 = perception.Gazebo.get_pose('block1')
            pose_list.append(pose_block1 == None)
            pose_kinect = perception.Gazebo.get_pose('kinect')
            pose_list.append(pose_kinect == None)
        except:            
            pass

        if all(pose_list) == False:
            break
            
    
    pose_table.position.z += 0.35
    pose_block0.position.z += 0.012
    pose_block1.position.z += 0.012
    pose_kinect.position.z += 0.036

    print(pose_table)
   

    while not rospy.is_shutdown():     
     
      
        rv.add_box("cafe_table", pose_table,model_size =(0.913,0.913,0.82))
        rv.add_box("block0", pose_block0,model_size =(0.025,0.025,0.025))
        rv.add_box("block1", pose_block1,model_size =(0.025,0.025,0.025))
        rv.add_box("kinect", pose_kinect,model_size =(0.073,0.276,0.072))

if __name__ == '__main__':
	main()

