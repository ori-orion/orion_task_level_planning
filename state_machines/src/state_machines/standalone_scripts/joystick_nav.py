#!/usr/bin/env python3

# import keyboard;
import sys;
import tty;
import select;
# import terminos;

import rospy;
import hsrb_interface;
import hsrb_interface.geometry as geometry
from hsrb_interface import robot as _robot

_robot.enable_interactive()

TRANSLATION_MAG = 0.1;
ROTATION_MAG = 0.1;

def echoOutput():
    while True:
        out = sys.stdin.read(1);
        print(out);
        # rospy.sleep(2);
        
def getKeyPressed():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def trackMotion():
    robot = hsrb_interface.Robot()
    # whole_body = robot.try_get("whole_body")
    omni_base = robot.try_get("omni_base")
    
    print(dir(omni_base));
    
    while True:
        delta_x = 0;
        delta_y = 0;
        delta_theta = 0;
        
        key_pressed = getKeyPressed();
        if len(key_pressed) == 0:
            rospy.sleep(0.01);
            continue;
        # print(key_pressed);
        
        if key_pressed == 'w':
            delta_x += TRANSLATION_MAG;
        if key_pressed == 's':
            delta_x -= TRANSLATION_MAG;
        if key_pressed == 'a':
            delta_y += TRANSLATION_MAG;
        if key_pressed == 'd':
            delta_y -= TRANSLATION_MAG;
        
        if key_pressed == 'q':
            delta_theta += ROTATION_MAG;
        if key_pressed == 'e':
            delta_theta -= ROTATION_MAG;
            
        if key_pressed == 'm':
            break;
            
        try:
            omni_base.follow_trajectory(
                [geometry.pose(x=delta_x, y=delta_y, ek=delta_theta)], 
                time_from_starts=[0.4], 
                ref_frame_id='base_footprint');
        except:
            pass;
        
        # omni_base.go_rel(delta_x, delta_y, delta_theta);
        
        rospy.sleep(0.1);

if __name__ == '__main__':
    
    rospy.init_node('joystick_nav');
    print("Node init");
    # echoOutput();
    trackMotion();
        
        
        