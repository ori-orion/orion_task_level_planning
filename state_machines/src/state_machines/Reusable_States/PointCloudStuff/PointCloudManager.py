import open3d as o3d;
import numpy as np;

import rospy;
import tf2_ros;
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

import utils;

# Currrent forms of data.
NUMPY = 1;
OPEN3D_PCL = 2;
OPEN3D_MESH = 3;
TORCH = 4;
BYTE_ARR = 5;

# Smoothing method.
NO_SMOOTHING = 0;


# Mesh making methods.
MESH_CREATION_BALL_PIVOTING = 1;
MESH_CREATION_POISSON = 2;
MESH_CREATION_METHOD = MESH_CREATION_POISSON;
MESH_CREATION_POISSON_DEPTH = 9;   # 9


class PointCloud:
    """
    Manages all the forms the point cloud might take.
    """
    def __init__(self, zero_append=3):
        self.data_np:np.ndarray = None;

        self.transformation_to_global_is_set:bool = False;
        self.transformation_to_global:np.ndarray = None;
    
        self.global_frame = "map";
        self.camera_frame = "head_rgbd_sensor_link";
        # self.camera_frame = "head_rgbd_sensor_gazebo_frame";
        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer);
    

    #region ROS interface
    def readROSPointCloud(self, point_cloud:PointCloud2):
        """
        Reads in the PointCloud2 object into the class.
        """
        self.data_form = NUMPY;
        self.nans_removed = False;
        self.row_step = int(point_cloud.row_step / point_cloud.point_step);

        array_form:np.ndarray = np.copy( np.frombuffer(point_cloud.data, dtype=np.float32).reshape((-1,int(point_cloud.point_step/4))) );
        array_form_shape = array_form.shape;
        self.data_np = np.zeros((array_form_shape[0],3));
        self.data_np[:,0:3] = array_form[:,0:3];

        if False:       # For including colour.
            rgb_float:np.ndarray = array_form[:,4];
            rgb_buffer = rgb_float.tobytes();
            rgb_np = np.copy( np.frombuffer( rgb_buffer, dtype=np.uint8 ).reshape((-1,4)) );
            r = rgb_np[:,2];
            g = rgb_np[:,1];
            b = rgb_np[:,0];
            self.data_np[:,3] = ( r.astype(np.float32) ) / 256;
            self.data_np[:,4] = ( g.astype(np.float32) ) / 256;
            self.data_np[:,5] = ( b.astype(np.float32) ) / 256;

        self.data_np = self.data_np.reshape((-1,self.row_step, 3));
    #endregion

    #region Coordinate system stuff.
    def getTransformationToGlobal(self, time:rospy.Time):
        camera_to_global:tf2_ros.TransformStamped = self.tfBuffer.lookup_transform(
            self.camera_frame, 
            self.global_frame, 
            time=time);
            # timeout=rospy.Duration(10));
        transformation_mat = np.eye(4);
        transformation_mat[0:3,0:3] = utils.quaternion_to_rot_mat(camera_to_global.transform.rotation);
        transformation_mat[0,3] = camera_to_global.transform.translation.x;
        transformation_mat[1,3] = camera_to_global.transform.translation.y;
        transformation_mat[2,3] = camera_to_global.transform.translation.z;
        return transformation_mat;
    
    def setSelfTransformation(self, time:rospy.Time):
        """
        Separate for odometry reasons.

        We want to call this only at the beginning of each odometry run so 
        that we can transform back into the frame we're expecting at the end.
        """
        if self.transformation_to_global_is_set == True:
            return;
        self.transformation_to_global = self.getTransformationToGlobal(time);
        self.transformation_to_global_is_set = True;
    #endregion

    def findClosestPoint(self, closest_point:np.ndarray):
        """

        """
        pass;
