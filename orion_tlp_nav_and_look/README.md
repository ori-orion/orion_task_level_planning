# Nav and Look package.

The main aim here is to, with a target location and orientation in mind, navigate around an object while looking at it to build up a slightly better point cloud of the object. 

This will mainly actually use the pair of stereo cameras on the front of the robot. 

Point clouds will be used for mapping. I would like this to be able to operate in unmapped environments as opposed to mapped ones.

## Overall ideas around design.

There's going to be a sequence of events here. 
 - Navigate into position
 - Look at the object and where we're going to need to navigate within a small distance of this. 
 - Take a picture.
 - Nav a little to the left
 - Take a picture
 - Nav a little to the right
 - Take a picture.
 - Execute Odometry pipeline on the images.
    - Open3d does not hav this capacity.


### Navigation

