import open3d as o3d;
import numpy as np;

import rospy;
import tf2_ros;
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
import geometry_msgs.msg;

from typing import List;

# Currrent forms of data.
NUMPY = 1;
OPEN3D_PCL = 2;
OPEN3D_MESH = 3;
TORCH = 4;
BYTE_ARR = 5;

# Smoothing method.
NO_SMOOTHING = 0;


def quaternion_to_rot_mat(quat:geometry_msgs.msg.Quaternion) -> np.ndarray:
    """
    Gets the 3x3 rotation matrix from a quaternion.
    https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    """
    output = np.zeros((3,3));

    quat_array = [quat.w, quat.x, quat.y, quat.z];
    quat_0 = quat_array[0];
    for i in range(0,3):
        quat_ip1 = quat_array[i+1];
        output[i,i] = 2 * (quat_0*quat_0 + quat_ip1*quat_ip1) - 1;
    
    # There's probably a nicer way to do this, but for now...
    output[0,1] = 2 * (quat_array[1]*quat_array[2] - quat_array[0]*quat_array[3]);
    output[1,0] = 2 * (quat_array[1]*quat_array[2] + quat_array[0]*quat_array[3]);

    output[0,2] = 2 * (quat_array[1]*quat_array[3] + quat_array[0]*quat_array[2]);
    output[2,0] = 2 * (quat_array[1]*quat_array[3] - quat_array[0]*quat_array[2]);

    output[1,2] = 2 * (quat_array[2]*quat_array[3] - quat_array[0]*quat_array[1]);
    output[2,1] = 2 * (quat_array[2]*quat_array[3] + quat_array[0]*quat_array[1]);
    
    return output;

class PointCloud:
    """
    Manages all the forms the point cloud might take.
    """
    def __init__(self):
        self.data_has_colour = True;

        self.data_np:np.ndarray = None;
        self.data_oped3d_pcl:o3d.geometry.PointCloud = None;

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
        self.data_np = np.zeros((array_form_shape[0], (6 if self.data_has_colour else 3)  ));
        self.data_np[:,0:3] = array_form[:,0:3];

        if self.data_has_colour:       # For including colour.
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
        transformation_mat[0:3,0:3] = quaternion_to_rot_mat(camera_to_global.transform.rotation);
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

    def findClosestPoint(self, closest_point:List[float]):
        """
        Finds the closest point in self.data_np 
        Inputs:
            closest_point:List[float]   : [x,y,z];  
        """

        pass;


    def filter_removeNanVals(self):
        self.data_np = self.data_np[ np.isnan(self.data_np[:,0]) == False, : ];
    def createO3dPclFromNumpy(self) -> o3d.geometry.PointCloud():
        """
        We want a method that doesn't modify the self here for odometry reasons.
        """
        self.filter_removeNanVals();
        data_open3d_pcl = o3d.geometry.PointCloud();
        data_open3d_pcl.points = o3d.utility.Vector3dVector(self.data_np[:,0:3]);
        if self.data_has_colour:
            data_open3d_pcl.colors = o3d.utility.Vector3dVector(self.data_np[:,3:6]);
        data_open3d_pcl.estimate_normals();
        data_open3d_pcl.orient_normals_towards_camera_location();
        return data_open3d_pcl;
    def numpyToO3dPcl(self):
        self.data_oped3d_pcl = self.createO3dPclFromNumpy();
    #region debugging stuff.
    def debug_visualisePointCloud(self):
        self.numpyToO3dPcl();
        
        o3d.visualization.draw_geometries([self.data_oped3d_pcl]);
    #endregion