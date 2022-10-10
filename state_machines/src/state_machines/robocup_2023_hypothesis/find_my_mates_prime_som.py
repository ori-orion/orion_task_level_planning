#!/usr/bin/env python3
"""
The main aim here is to be able to test this in siulation.
In order to do this, we will prime the SOM system with fake humans to interact with.
"""

from geometry_msgs.msg import Pose, PoseStamped;
from orion_actions.srv import *;
import rospy;

def primeSOM():
    
    operator_pose = Pose();
    operator_pose.position.x = 4.029820256813819;
    operator_pose.position.y = -0.5829549579052932;
    operator_pose.position.z = 0.0;
    operator_pose.orientation.x = 0.0
    operator_pose.orientation.y = 0.0
    operator_pose.orientation.z = 0.9690647903801627
    operator_pose.orientation.w = 0.24680646678207457

    guest_1_pose = Pose();
    guest_1_pose.position.x = 4.043979671699259;
    guest_1_pose.position.y = 0.7322083071752444;
    guest_1_pose.position.z = 0.0;
    guest_1_pose.orientation.x = 0.0
    guest_1_pose.orientation.y = 0.0
    guest_1_pose.orientation.z = -0.9859053137702275
    guest_1_pose.orientation.w = 0.1673042506322813

    guest_2_pose = Pose();
    guest_2_pose.position.x = 0.8788947178395814;
    guest_2_pose.position.y = 0.9410268283731436;
    guest_2_pose.position.z = 0.0;
    guest_2_pose.orientation.x = 0.0
    guest_2_pose.orientation.y = 0.0
    guest_2_pose.orientation.z = -0.44044671674760205
    guest_2_pose.orientation.w = 0.8977787532049628

    rospy.init_node('find_my_mates_prime_som');

    rospy.wait_for_service('/som/human_observations/input');
    human_obs_srv = rospy.ServiceProxy('/som/human_observations/input', SOMAddHumanObs);
    counter = 0;
    def push_to_service(pose_in:Pose):
        adding = SOMAddHumanObsRequest();
        adding.adding.obj_position = pose_in;
        adding.adding.observed_at = rospy.Time.now();
        adding.adding.object_uid = "prime_" + str(counter);
        human_obs_srv(adding);

    push_to_service(operator_pose);
    counter += 1;
    push_to_service(guest_1_pose);
    counter += 1;
    push_to_service(guest_2_pose);
    counter += 1;

primeSOM();