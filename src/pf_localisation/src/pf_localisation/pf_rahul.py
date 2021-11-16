from geometry_msgs.msg import Pose, PoseArray, Quaternion
from .pf_base import PFLocaliserBase
import math
import rospy

from .util import rotateQuaternion, getHeading
import random
import numpy as np

from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0  # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0  # Odometry y axis (side-side) noise
        self.SIGMA = 0.01

        #used to generate initial partical cloud
        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.15

        # ----- Sensor model parameters
        # empty pose initially
        self.robot_pose = Pose()
        # initial number of particles generated
        self.number_particles = 500

        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

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

        self.robot_pose = initialpose
        # pose array has the poses of all particles
        pose_list = PoseArray()
        for i in range(self.number_particles):

            particle_pose = Pose()

            # calculating the noises
            self.ODOM_TRANSLATION_NOISE = np.random.normal(particle_pose.position.x, self.SIGMA**2)
            self.ODOM_DRIFT_NOISE = np.random.normal(particle_pose.position.y, self.SIGMA**2)
            self.ODOM_ROTATION_NOISE = np.random.normal(particle_pose.orientation.z, self.SIGMA**2)

            particle_pose.position.x = self.robot_pose.pose.pose.position.x + self.ODOM_TRANSLATION_NOISE
            particle_pose.position.y = self.robot_pose.pose.pose.position.y + self.ODOM_DRIFT_NOISE
            particle_pose.orientation.z = 1  # has to be randomized

            # set quaternions (the robot only rotates around yaw axis ("heading"))
            # multiply random with 2*pi to get anything from 0 to 2pi radians in rotation
            # particle_pose.orientation = rotateQuaternion(particle_pose.orientation, random.random()*(math.pi*2))
            # particle_pose.orientation.w

            pose_list.poses.append(particle_pose)

        # the particles should also have uniformly distributed 1/n weights

        # returning the poses of the particles
        return pose_list

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
       
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        # normalization
        normalized_weight = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses]
        for i in range(self.number_particles):
            normalized_weight[i] /= sum(normalized_weight)

        new_sample = PoseArray()  # step 1 of systematic resample algo
        cumulative_distribution = [0 for _ in range(self.number_particles)]
        cumulative_distribution[0] = normalized_weight[0]
        for i in range(1, self.number_particles):
            # adding the contribution for particle i
            # if all particles are normalized then the last contribution has to be 1
            weight_particle_i = normalized_weight[i]
            rospy.loginfo(f"The weight for particle {i} is {weight_particle_i}")
            cumulative_distribution[i] = cumulative_distribution[i - 1] + weight_particle_i

        # ----threshold step -----
        u = random.random() * (1 / self.number_particles) + 0.0000001
        rospy.loginfo(f"U is {u} ")

        i = 0
        for j in range(self.number_particles):
            while u > cumulative_distribution[i]:  # u_i > c_i
                i += 1
            new_particle = Pose()

            # calculating the noises
            self.ODOM_TRANSLATION_NOISE = np.random.normal(self.particlecloud.poses[i].position.x, self.SIGMA**2)
            self.ODOM_DRIFT_NOISE = np.random.normal(self.particlecloud.poses[i].position.y, self.SIGMA**2)
            self.ODOM_ROTATION_NOISE = np.random.normal(self.particlecloud.poses[i].orientation.z, self.SIGMA**2)

            new_particle.position.x = self.particlecloud.poses[i].position.x + self.ODOM_TRANSLATION_NOISE
            new_particle.position.y = self.particlecloud.poses[i].position.y + self.ODOM_DRIFT_NOISE
            new_particle.orientation.z = 1 # + self.ODOM_ROTATION_NOISE
            new_sample.poses.append(new_particle)  # picked the good particle
            u += 1 / self.number_particles


        self.particlecloud = new_sample
        self.particlecloud.header.frame_id = 'map'


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

        # worst solution ever but it is a good one
        estimate_pose = Pose()
        position_x, position_y = 0, 0
        particles_heading = 0
        for particle in self.particlecloud.poses:
            position_x += particle.position.x
            position_y += particle.position.y
            particles_heading += particle.orientation.z

        estimate_pose.position.x = position_x / self.number_particles
        estimate_pose.position.y = position_y / self.number_particles
        estimate_pose.orientation.z = (particles_heading / self.number_particles) % (2 * math.pi)

        return estimate_pose