import os
import sys
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from numpy.random.mtrand import vonmises
import numpy as np

from . pf_base import PFLocaliserBase
import math
import rospy
import tf 

from . util import rotateQuaternion, getHeading
from random import random, gauss
from time import time


class PFLocaliser(PFLocaliserBase):
    """ Subclass of PFLocaliserBase, implement all un-implemented functions! """

    def __init__(self):

        super(PFLocaliser, self).__init__()
        
        self.NUMBER_PARTICLES = 100
        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.15
        self.ODOMETRIC_DRIFT_NOISE = 0  
        self.ODOMETRIC_ROTATION_NOISE = 0       
        self.ODOMETRIC_TRANSLATION_NOISE = 0

        self.NUMBER_PREDICTED_READINGS = 20  
                

    def get_position(self, coordinate):
        
        # Cell location will be determined given a point
        resolution = self.occupancy_map.info.resolution
        return (int(math.floor(coordinate.x / resolution)), int(math.floor(coordinate.y / resolution)))


    def is_valid_position(self, coordinate):
        # Predicts a valid location for the robot
        if coordinate is None:
            return False
        
        (x, y) = self.get_position(coordinate)
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        index = x + y * width

        invalid = (index > len(self.occupancy_map.data) or
                     x >= width or x < 0 or y >= height or y < 0)
        value = -1 if invalid else self.occupancy_map.data[index]

        return value != -1

    def random_position_generator(self):
        # Generates a random position.

        p = Pose()  
        
        while not self.is_valid_position(p.position):
            p.position.x = random() * self.occupancy_map.info.width 
            p.position.y = random() * self.occupancy_map.info.height  
        p.position.z = 0.0  

        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0

        p.orientation = rotateQuaternion(p.orientation, 2 * math.pi * random())

        return p



    def initialise_particle_cloud(self, initial_pose):
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
        

        def gauss_pose_generator(initial_pose):
            p = Pose()
            p.position.x = gauss(
                initial_pose.pose.pose.position.x, self.POSITION_STANDARD_DEVIATION)
            p.position.y = gauss(
                initial_pose.pose.pose.position.y, self.POSITION_STANDARD_DEVIATION)
            p.position.z = 0.0

            rotation_angle = vonmises(
                0.0, 1 / self.ORIENTATION_STANDARD_DEVIATION ** 2)

            p.orientation = rotateQuaternion(
                initial_pose.pose.pose.orientation, rotation_angle)

            return p

        poses = [gauss_pose_generator(initial_pose)
                 for i in range(self.NUMBER_PARTICLES)]



        pose_array = PoseArray()
        for pose in poses:
            pose_array.poses.append(pose)
        return pose_array




    def update_particle_cloud(self, scan):
        """ Update particle cloud given laser scan.
        This function requires using the sensor model to compute the likelihoods
        of particles (odometry prediction is already written and used
        elsewhere).
        Before computing this, you may want to add some Gaussian noise to the 
        particles due to noise in the action model. If you do this, make sure it
        is a valid position though!
        Given the likelihoods, you then want to resample a new set of particles.
        You may also wish to add a small amount of Gaussian noise to the newly
        generated particles.
        Note that you will need to use the sensor model here.
        The function you will want to compute the likelihood is
        self.sensor_model.get_weight(scan, particle).
        This function should set self.particlecloud.poses to the new pose list.
        Args:
            scan: The LaserScan message
        """

        def noise_distribution(pos, yaw):
            poses_array = []
            for p in self.particlecloud.poses:  # Iterate over all poses
                new_pose = Pose()
                # Repeat until point is valid (unoccupied)
                while not self.is_valid_position(new_pose.position):
                    # Add noise to x-coordinate
                    new_pose.position.x = gauss(p.position.x, pos)
                    # Add noise to y-coordinate
                    new_pose.position.y = gauss(p.position.y, yaw)

                rotate_amounts = vonmises(0.0, 1 / yaw ** 2)
                new_pose.orientation = rotateQuaternion(
                    p.orientation, rotate_amounts)

                poses_array.append(new_pose)
            self.particlecloud.poses = poses_array

        STANDARD_POS_1 = 0.1  
        STANDARD_POS_2 = 0.1
        STANDARD_YAW_1 = 1.0 / 180.0 * math.pi
        STANDARD_YAW_2 = 1.0 / 180.0 * math.pi
        
        noise_distribution(STANDARD_POS_1, STANDARD_YAW_1)

        
        likelihood = np.array([self.sensor_model.get_weight(scan, particle) for particle in
                               self.particlecloud.poses])  

        w = likelihood / sum(likelihood) 

        def data(arr, w):
            resampled_array = []
            n = self.NUMBER_PARTICLES
            cummulative_sum_of_weights = [0.0] + [np.sum(w[: i + 1]) for i in range(n)]
            u0 = np.random.random()
            j = 0

            for u in [(u0 + i) / n for i in range(n)]:
                while u > cummulative_sum_of_weights[j]:
                    j += 1
                resampled_array.append(arr[j - 1])
            return resampled_array
        
        self.particlecloud.poses = data(self.particlecloud.poses, w)

        for i in range(40):
            self.particlecloud.poses[i] = self.random_position_generator()

        noise_distribution(STANDARD_POS_2, STANDARD_YAW_2)



    def estimate_pose(self):
        """ Create new estimated pose, given particle cloud.
        E.g. just average the location and orientation values of each of
        the particles and return this.
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers.
        Remember that your particle cloud poses are stored in
        self.particlecloud.poses
        Return:
            pose: The estimated pose (should be a geometry_msgs.Pose message)
        """
        estimate = Pose()


        for i in ['x', 'y', 'z']:
            mean = np.mean([getattr(pose.position, i)
                            for pose in self.particlecloud.poses])
            setattr(estimate.position, i, mean)


        def orientation_to_vector(orientation):
            return [getattr(orientation, i) for i in ['x', 'y', 'z', 'w']]

        def vector_to_orientation(vec):
            orientation = Quaternion()
            orientation.x, orientation.y, orientation.z, orientation.w = vec
            return orientation
        

        def clustering(poses):
            vector_arr = [np.array(orientation_to_vector(pose.orientation))
                 for pose in poses]
            zero_arr = np.zeros((4, 4))  
            arr_len = len(vector_arr)
            for q in vector_arr:
                if q[0] < 0:  
                    q = -q
                zero_arr = np.outer(q, q) + zero_arr 

            zero_arr = (1.0 / arr_len) * zero_arr  
            

            values, vectors = np.linalg.eig(zero_arr)
            maxcol = values.argmax()
            Q_avg = vectors[:, maxcol]
            return vector_to_orientation(Q_avg)


        estimate.orientation = clustering(self.particlecloud.poses)
        

        return estimate
