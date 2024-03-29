from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;

from geometry_msgs.msg import Pose, PoseStamped;

# from orion_face_recognition.msg import ActionServer_CapFaceAction, ActionServer_CapFaceGoal, \
#     ActionServer_FindMatchAction, ActionServer_FindMatchGoal, ActionServer_FindAttrsAction, ActionServer_FindAttrsGoal, \
#         ActionServer_ClearDatabaseAction, ActionServer_ClearDatabaseGoal

import actionlib;

FACE_ATTRIBUTES_EXCLUSION_LIST = ['Attractive',    'Bags_Under_Eyes',    'Bald',    'Big_Lips',    'Big_Nose', 'Blurry', 'Chubby',    'Double_Chin', 'Heavy_Makeup',    'Mouth_Slightly_Open',    'Narrow_Eyes',    'Pale_Skin',    'Pointy_Nose',    'Receding_Hairline', 'Smiling',  'Young']
def filter_face_attributes_by_exclusion(attributes):
    """ Filter the list of facial attributes by removing items in the exclusion list. """
    filtered_attributes = [attr for attr in attributes if attr not in FACE_ATTRIBUTES_EXCLUSION_LIST]

    return filtered_attributes

#region Facial stuff

# class RegisterFace(SmachBaseClass):
#     """ State for robot to register a new face using facial capture action server

#     The action server detects faces from the robot RGB-D sensor's RGB camera image topic

#     input_keys:
#         face_id: (optional) the name or identifier of the person whose face is being registered
#     output_keys:
#         registered_face_id: the id assigned to the registered face. This will be the input face_id if given,
#                             else a unique sequential id generated by the facial capture action server
#     """

#     def __init__(self):
#         SmachBaseClass.__init__(self, outcomes=[SUCCESS,FAILURE],
#                                 input_keys=['face_id'],
#                                 output_keys=['registered_face_id'])

#     def execute(self, userdata):
#         # if the face_id is not given, then set it to an empty string;
#         # the capface server will automatically generate one for us
#         if("face_id" not in userdata):
#             face_id = ""
#         else:
#             face_id = userdata["face_id"]

#         # set action goal and call action server
#         capface_goal = ActionServer_CapFaceGoal(face_id="");

#         capface_action_client = actionlib.SimpleActionClient('as_Capface', ActionServer_CapFaceAction)
#         capface_action_client.wait_for_server()
#         rospy.loginfo("Calling CapFace action server...")
#         capface_action_client.send_goal(capface_goal)
#         capface_action_client.wait_for_result()

#         # process result
#         found_face = capface_action_client.get_result().If_saved
#         if not found_face:
#             # no faces were found in the image
#             rospy.logwarn("Capface action server failed. Did not find any faces in the image")
#             return "failure"
#         # success
#         registered_face_id = capface_action_client.get_result().name
#         userdata["registered_face_id"] = registered_face_id
#         rospy.loginfo("Capface action server registered face with id: {}".format(registered_face_id))
#         return "success"

# class RecogniseFace(SmachBaseClass):
#     """ State for robot to recognise a face and match it to a previously registered face,
#         using the facial capture action server

#     input_keys:
#         min_score_threshold: (optional) the minimum score for a matched face to be considered a valid match
#     output_keys:
#         face_id: the id of the detected face, used in the facelib system / capface action server
#         face_match_score: the match score of the recognised face
#     """

#     def __init__(self):
#         SmachBaseClass.__init__(self, outcomes=[SUCCESS, FAILURE],
#                                 input_keys=['min_score_threshold'],
#                                 output_keys=['face_id','face_match_score'])

#     def execute(self, userdata):
#         # use min_score_threshold, if set
#         if "min_score_threshold" in userdata:
#             min_score_threshold = userdata["min_score_threshold"]
#         else:
#             min_score_threshold = 0.1

#         # set action goal and call action server
#         findmatch_goal = ActionServer_FindMatchGoal()

#         findmatch_action_client = actionlib.SimpleActionClient('as_Findmatch', ActionServer_FindMatchAction)
#         findmatch_action_client.wait_for_server()
#         rospy.loginfo("Calling face match action server...")
#         findmatch_action_client.send_goal(findmatch_goal)
#         findmatch_action_client.wait_for_result()

#         # process result
#         if(not findmatch_action_client.get_result().If_find):
#             # face not found
#             rospy.loginfo("Face FindMatch action server did not find any faces in the image")
#             return "failure"
#         # face was found
#         matched_face_id = findmatch_action_client.get_result().face_id
#         matched_face_score = findmatch_action_client.get_result().best_match_score
#         # matched_face_file_name = findmatch_action_client.get_result().file_name
#         if(matched_face_score < min_score_threshold):
#             # no faces found with high enough score/confidence
#             rospy.logwarn("Face FindMatch action server failed. Did not find any matches above min score threshold ({}); best score is '{}'.".format(min_score_threshold, matched_face_score))
#             return "failure"
#         # result was good!
#         userdata["face_id"] = matched_face_id
#         userdata["face_match_score"] = matched_face_score
#         rospy.loginfo("Face FindMatch action server recognised face_id '{}', face_score: {}".format(matched_face_id, matched_face_score))
#         return "success"

# class DetectFaceAttributes(SmachBaseClass):
#     """ State for the robot to detect face attributes

#     Uses the FaceLib FindAttrs action server.
#     Always succeeds; an empty attributes list is not a failure.

#     input_keys:
#         face_id: (optional) If given, the action server will analyse the saved face for this face_id,
#                             otherwise it will analyse the face that currently appears in the image topic
#     output_keys:
#         face_attributes: A list of detected facial attributes, represented as strings.
#                          If a certain feature is detected, it will be present in this list.
#                          Thus, features not in the list were not detected.
#         num_attributes: The number of detected facial attributes
#     """

#     def __init__(self):
#         SmachBaseClass.__init__(self, outcomes=[SUCCESS],
#                                 input_keys=['face_id'],
#                                 output_keys=['face_attributes', 'num_attributes'])

#     def execute(self, userdata):
#         # if the face_id is not given, then set it to an empty string;
#         # the capface server will use the live camera topic feed (rather than the pre-registered face)
#         if("face_id" not in userdata):
#             face_id = ""
#         else:
#             face_id = userdata["face_id"]

#         # set action goal and call action server
#         find_face_attrs_goal = ActionServer_FindAttrsGoal()

#         find_face_attrs_action_client = actionlib.SimpleActionClient('as_Findattrs', ActionServer_FindAttrsAction)
#         find_face_attrs_action_client.wait_for_server()
#         rospy.loginfo("Calling find face attributes action server...")
#         find_face_attrs_action_client.send_goal(find_face_attrs_goal)
#         find_face_attrs_action_client.wait_for_result()

#         # process result
#         face_attributes_raw = find_face_attrs_action_client.get_result().attrs
#         num_attributes_raw = find_face_attrs_action_client.get_result().num_attrs
#         face_attributes = filter_face_attributes_by_exclusion(face_attributes_raw)         # remove unwanted labels
#         num_attributes = len(face_attributes)
#         userdata["face_attributes"] = face_attributes
#         userdata["num_attributes"] = num_attributes
#         rospy.loginfo("FaceAttributes action server registered {} face attibutes: \n\t{}".format(num_attributes, face_attributes))
#         # rospy.loginfo("FaceAttributes action server registered {} face attibutes: \n\t{}\nFound {} unfiltered attributes: \n\t{}".format(num_attributes, face_attributes, num_attributes_raw, face_attributes_raw))
#         return "success"

# class ClearFaceDB(SmachBaseClass):
#     """ State for the robot to clear the face database

#     Uses the FaceLib as_Cleardatabase action server

#     input_keys:

#     output_keys:

#     """

#     def __init__(self):
#         SmachBaseClass.__init__(self, outcomes=[SUCCESS])

#     def execute(self, userdata):
#         # set action goal and call action server
#         clear_face_db_goal = ActionServer_ClearDatabaseGoal()

#         clear_face_db_action_client = actionlib.SimpleActionClient('as_Cleardatabase', ActionServer_ClearDatabaseAction)
#         clear_face_db_action_client.wait_for_server()
#         rospy.loginfo("Calling clear face db action server...")
#         clear_face_db_action_client.send_goal(clear_face_db_goal)
#         clear_face_db_action_client.wait_for_result()

#         # process result
#         # is_success = clear_face_db_action_client.get_result().Is_success - is this used? it's not essential atm

#         rospy.loginfo("ClearFaceDatabase action server cleared the database")
#         return "success"

#endregion
