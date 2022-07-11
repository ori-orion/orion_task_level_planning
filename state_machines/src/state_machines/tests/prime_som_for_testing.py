#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Point;
from orion_actions.srv import SOMAddObservation, SOMAddObservationRequest;

import rospy;

def create_human_observation(observed_at:Pose, size:Point=Point()) -> SOMAddObservationRequest:
    obs = SOMAddObservationRequest();

    obs.adding.obj_position = observed_at;
    obs.adding.class_ = "person";
    obs.adding.observed_at = rospy.Time.now();
    obs.adding.observation_batch_num = 1;
    obs.adding.size = size;

    return obs;

def prime_SOM_system():
    rospy.init_node("iuawfh");
    typical_size = Point();
    typical_size.x = 0.1;
    typical_size.y = 0.1;
    typical_size.z = 2.5;

    som_observation_input_srv = rospy.ServiceProxy('/som/observations/input', SOMAddObservation);

    """
    pose: 
      position: 
        x: 5.847205116292954
        y: -0.06664083269901128
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.9353678902554121
        w: 0.35367627836644544
    """
    pose_1 = Pose();
    pose_1.position.x = 5.847205116292954;
    pose_1.position.y = -0.06664083269901128;
    pose_1.position.z = 0.0;
    pose_1.orientation.x = 0.0;
    pose_1.orientation.y = 0.0;
    pose_1.orientation.z = 0.9353678902554121;
    pose_1.orientation.w = 0.35367627836644544;
    som_observation_input_srv(create_human_observation(pose_1, typical_size));

    """
    pose: 
      position: 
        x: 2.5660398068352572
        y: 4.79595756974329
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: -0.9245816979164214
        w: 0.38098383676737146
    """
    pose_2 = Pose();
    pose_2.position.x = 2.5660398068352572;
    pose_2.position.y = 4.79595756974329;
    pose_2.position.z = 0.0;
    pose_2.orientation.x = 0.0;
    pose_2.orientation.y = 0.0;
    pose_2.orientation.z = -0.9245816979164214;
    pose_2.orientation.w = 0.38098383676737146;
    som_observation_input_srv(create_human_observation(pose_2, typical_size));

    """
    pose: 
      position: 
        x: 0.8692771064552061
        y: 3.5180767013223746
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.025698261608417518
        w: 0.9996697451410167
    """
    pose_3 = Pose(); 
    pose_3.position.x = 0.8692771064552061;
    pose_3.position.y = 3.5180767013223746;
    pose_3.position.z = 0.0;
    pose_3.orientation.x = 0.0;
    pose_3.orientation.y = 0.0;
    pose_3.orientation.z = 0.025698261608417518;
    pose_3.orientation.w = 0.9996697451410167;
    som_observation_input_srv(create_human_observation(pose_3, typical_size));

if __name__ == '__main__':
    prime_SOM_system();