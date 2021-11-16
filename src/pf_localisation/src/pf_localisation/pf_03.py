from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from . pf_base import PFLocaliserBase
import math
import numpy
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE =  0
        self.ODOM_TRANSLATION_NOISE = 0
        self.ODOM_DRIFT_NOISE = 0
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 100     # Number of readings to predict default = 20
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        ip = initialpose
        ip_pose = ip.pose.pose
        pose_array = PoseArray()
        gaus_gen = numpy.random.default_rng()
        for x in range(1,self.NUMBER_PREDICTED_READINGS):
        	gaus = gaus_gen.normal(0,1,3)
        	new_pose = Pose()
        	new_pose.position.x = ip_pose.position.x + gaus[0]
        	new_pose.position.y = ip_pose.position.y + gaus[1]
        	new_pose.orientation = rotateQuaternion(ip_pose.orientation, gaus[2] * math.pi)
        	pose_array.poses.append(new_pose)
        return pose_array
        pass

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
	
         """
        
        total_weight = 0
        weight_array = []
        weight_array = [self.sensor_model.get_weight(scan, pose) for pose in self.particlecloud.poses]
        total_weight = sum(weight_array)
        pose_probabilities = [pose_weight/total_weight for pose_weight in weight_array]
        choice = numpy.random.choice(self.particlecloud.poses, None, True, pose_probabilities)
        new_pose = PoseWithCovarianceStamped()
        new_pose.pose.pose = choice
        self.particlecloud = self.initialise_particle_cloud(new_pose)
        self.particlecloud.header.frame_id = "map"
        pass


    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        return Pose()
        pass    