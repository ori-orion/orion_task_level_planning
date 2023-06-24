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

    #region General stuff
    def getPointwiseDistMeasure(self):
        """
        Roughly how close are the points to each other.
        Looks at adjacent points and takes the average over their distances.
        """
        data_shape = self.data_np.shape;
        pointwise_deltas_horizontal = self.data_np[0:data_shape[0]-1,:,0:3] - self.data_np[1:data_shape[0],:,0:3];
        pointwise_deltas_vertical   = self.data_np[:,0:data_shape[1]-1,0:3] - self.data_np[:,1:data_shape[1],0:3];

        dist_horiz:np.ndarray = np.sqrt( np.sum( pointwise_deltas_horizontal*pointwise_deltas_horizontal, axis=2) );
        dist_vert:np.ndarray  = np.sqrt( np.sum( pointwise_deltas_vertical*pointwise_deltas_vertical, axis=2) );

        summed_dist = dist_horiz.sum() + dist_vert.sum();
        tot_num_elements = dist_horiz.shape[0]*dist_horiz.shape[1] + dist_vert.shape[0]*dist_vert.shape[1];

        return summed_dist / tot_num_elements;
    #endregion

    #region Cropping the point cloud.
    def findClosestPoint(self, tf_point:List[float]) -> tuple:
        """
        Finds the closest point in self.data_np 
        Inputs:
            tf_point:List[float]   : [x,y,z], Needs to be in the frame of the camera. (We're not transforming everything to global).
        Operates on:
            self.data_np:np.ndarray[height,width,3|6]
        """
        data_copy:np.ndarray = self.data_np.copy();
        for i in range(3):
            data_copy[:,:,i] -= tf_point[i];
        squared_distances:np.ndarray = np.sum( data_copy[:,:,0:3]*data_copy[:,:,0:3], axis=2 );
        min_index = np.unravel_index( np.argmin(squared_distances, axis=None), squared_distances.shape );
        print("Expecting this to be a 2-tuple");
        print("Minimum index is", min_index);
        return min_index;
    def getPointsInImageCloseToClosestPoint(self, tf_point:List[float]):
        """
        Removes points in the image space that are too far away. This reduces the processing required.
        """
        # Thinking about this as an image, how many points in that image to the left, right, up and down are we going to take.
        POINT_DELTA = 50;

        closest_point_index:tuple = self.findClosestPoint(tf_point);
        left = closest_point_index[0] - POINT_DELTA;
        right = closest_point_index[0] + POINT_DELTA;
        bottom = closest_point_index[1] - POINT_DELTA;
        top = closest_point_index[1] + POINT_DELTA;

        if left < 0:
            left = 0;
        if right >= self.data_np.shape[0]:
            right = self.data_np.shape[0]-1;
        if bottom < 0:
            bottom = 0;
        if top >= self.data_np.shape[1]:
            top = self.data_np.shape[1]-1;
        
        self.data_np = self.data_np[ left:right, bottom:top, :];
    #endregion

    #region Finding a plane:
    """
    For this we will use RANSAC.
    We are assuming the largest plane is the table. This could be a bad assumption in some cases, but it should hold in most.
    The whole aim of this is to simply remove these points. We can thus set them to Nan when we're done. I don't think we will
    need them again if we have the model of the plane.

    Model of the plane:
    n.x=n.a or ax+by+cz+d=0
    Ideally we would have a parallelisable method for working out the distances.
    n.(p-a) = |p-a| cos theta
            = d
    where p is the point, a is a point on the plane, and n is a unit vector.
    This should then be parallelisable.
    n we find using cross products.
    """
    def RANSAC_getRandomPoints(self, num=3) -> np.ndarray:
        """
        Returns a num x 3 array with each column being a point. 
        """
        i0 = np.random.randint(0, self.data_np.shape[0], size=(num,));
        i1 = np.random.randint(0, self.data_np.shape[1], size=(num,));

        output = np.ndarray((num,3));
        for i in range(num):
            output[i,:] = self.data_np[i0[i], i1[i], 0:3];
        return output;
    def RANSAC_getNormalVec(self, rand_points:np.ndarray) -> np.ndarray:
        """
        Gets the normal vector using cross products.
        Returns normalised vec.
        rand_points:np.ndarray is in the same form as from RANSAC_getRandomPoints(...);
        """
        delta_1 = rand_points[0,:] - rand_points[1,:];
        delta_2 = rand_points[0,:] - rand_points[2,:];
        normal = np.cross(delta_1, delta_2);
        normal /= np.linalg.norm(normal);
        return normal;
    def RANSAC_PlaneAlg(self):
        """
        Runs RANSAC on the point cloud to find a plane.
        Note, it will then set the matching points to NAN, thus removing them and allowing for us to progress.
        """
        MAX_NUM_ITS = 200;
        MIN_PROPORTION = 0.3;
        MEAN_DIST_MULT_FOR_PLANE_DIST_THRESHOLD = 2;

        mean_dist_between_adjacent_points = self.getPointwiseDistMeasure();
        plane_dist_threshold:float = MEAN_DIST_MULT_FOR_PLANE_DIST_THRESHOLD*mean_dist_between_adjacent_points;

        data_in_shape = self.data_np.shape;
        total_num_points = data_in_shape[0] * data_in_shape[1];

        best_proportion = 0;
        best_matches = None;

        for i in range(MAX_NUM_ITS):
            rand_points = self.RANSAC_getRandomPoints();
            normal_vec = self.RANSAC_getNormalVec(rand_points);

            pointwise_deltas = np.ndarray(self.data_np.shape);
            pointwise_distances = np.zeros( (data_in_shape[0],data_in_shape[1]) );
            for i in range(3):
                pointwise_deltas[:,:,i] = self.data_np[:,:,i] - rand_points[0,i];
                pointwise_distances[:,:] += pointwise_deltas[:,:,i] * normal_vec[i];

            match_plane = pointwise_distances < plane_dist_threshold;
            num_matches = np.sum(match_plane);

            proportion_matching = num_matches/total_num_points;

            if proportion_matching > best_proportion:
                best_proportion = proportion_matching;
                best_matches = match_plane;

            if proportion_matching > MIN_PROPORTION:
                break;


        pass;
    #endregion


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