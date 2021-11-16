from geometry_msgs.msg import Pose, PoseArray, Quaternion
from numpy.random.mtrand import vonmises
from . pf_base import PFLocaliserBase
import math
import numpy as np
import rospy

from . util import rotateQuaternion, getHeading
from random import random, gauss

from time import time

def orientation_to_vec(orientation):
    return [getattr(orientation, k) for k in ['x', 'y', 'z', 'w']]


def vec_to_orientation(vec):
    o = Quaternion()
    o.x, o.y, o.z, o.w = vec
    return o

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0        # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0     # Odometry x axis (fwd) noise
        self.ODOM_DRIFT_NOISE = 0  # Odometry y axis (side-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.NUMBER_PARTICLES = 50

        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.15

    def generate_random_pose(self):
        """ Generates a random pose on the map.
        Return:
            (geometry_msgs.msg.Pose) the random pose on the map
        """
        p = Pose()  # Instantiate pose
        while not self.is_valid_position(p.position):  # Repeat until unoccupied point is found
            p.position.x = random() * self.occupancy_map.info.width  # Sample x location on map
            p.position.y = random() * self.occupancy_map.info.height  # Sample y location on map
        p.position.z = 0.0  # No elevation, z-coordinate is 0

        p.orientation.w = 1.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation = rotateQuaternion(p.orientation, 2 * math.pi * random())

        return p
       
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
        p = [self.generate_random_pose() for i in range(self.NUMBER_PARTICLES)]  # Must use PoseArray() instead?


        def gen_gauss_pose(self, initialpose):

            p = Pose()
            p.position.x = gauss(initialpose.pose.pose.position.x, self.POSITION_STANDARD_DEVIATION)
            p.position.y = gauss(initialpose.pose.pose.position.y, self.POSITION_STANDARD_DEVIATION)
            p.position.z = 0.0

            rotate_amounts = vonmises(0.0, 1 / self.ORIENTATION_STANDARD_DEVIATION ** 2)
            p.orientation = rotateQuaternion(initialpose.pose.pose.orientation, rotate_amounts)

            return p

        poses = [gen_gauss_pose(self, initialpose) for i in range(self.NUMBER_PARTICLES)]

        pose_array = PoseArray()

        for pose in poses:
            pose_array.poses.append(pose)
        
        return pose_array

    def point_to_cell(self, point):
        """ Converts a point into a cell location.
        Args:
            point (geometry_msgs.msg.Point) point using meter units
        Return:
            (int, int) cell location on map
        """
        res = self.occupancy_map.info.resolution
        return (int(math.floor(point.x / res)), int(math.floor(point.y / res)))

    def is_valid_position(self, point):
        """ Checks whether a point is a valid location for the robot on the map.
        Args:
            point (geometry_msgs.msg.Point) point using meter units
        Return:
            (boolean) whether or not the robot can be in that position
        """
        if point is None: return False
        (x, y) = self.point_to_cell(point)
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        index = x + y * width

        # data is row major
        not_valid = (index > len(self.occupancy_map.data) or
                     x >= width or x < 0 or y >= height or y < 0)
        value = -1 if not_valid else self.occupancy_map.data[index]
        return value != -1

    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
         """
        
        def add_noise(std_pos, std_yaw):
            poses = []
            for p in self.particlecloud.poses:
                new_pose = Pose()
                while not self.is_valid_position(new_pose.position):
                    new_pose.position.x = gauss(p.position.x, std_pos)
                    new_pose.position.y = gauss(p.position.y, std_pos)

                rotate_amounts = vonmises(0.0, 1/(std_yaw)**2)
                new_pose.orientation = rotateQuaternion(p.orientation, rotate_amounts)

                poses.append(new_pose)
            self.particlecloud.poses = poses

        std_pos_1 = 0.1
        std_yaw_1 = 1.0 / 180.0 * math.pi
        std_pos_2 = 0.1
        std_yaw_2 = 1.0 / 180.0 * math.pi

        add_noise(std_pos=std_pos_1, std_yaw=std_yaw_1)
        likelihood = np.array([self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses])
        w = likelihood / sum(likelihood)

        def resample(array, weights):
            resampled = []
            n = self.NUMBER_PARTICLES
            cum_sum_weights = [0.0] + [np.sum(weights[:i+1]) for i in range(n)]
            u0 = np.random.random()
            j = 0
            for u in [(u0 + i) / n for i in range(n)]:
                while u > cum_sum_weights[j]:
                    j += 1
                resampled.append(array[j - 1])
            return resampled

        self.particlecloud.poses = resample(self.particlecloud.poses, w)

        for i in range(40):
            self.particlecloud.poses[i] = self.generate_random_pose()

        add_noise(std_pos = std_pos_2 , std_yaw = std_yaw_2)

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

        estimate = Pose()

        for k in ['x', 'y', 'z']:
            mean = np.mean([getattr(pose.position, k) for pose in self.particlecloud.poses])
            setattr(estimate.position, k, mean)

        def avg_quaternion(poses):
            Q = [np.array(orientation_to_vec(pose.orientation)) for pose in poses]
            A = np.zeros((4, 4))
            M = len(Q)

            for q in Q:
                if q[0] < 0:
                    q = -q
                A = np.outer(q, q) + A
            
            A = (1.0 / M) * A
            vals = np.linalg.eig(A)
            vects = np.linalg.eig(A)
            maxcol = vals.argmax()
            Q_avg = vects[:, maxcol]
            return vec_to_orientation(Q_avg)

        estimate.orientation = avg_quaternion(self.particlecloud.poses)
        return estimate