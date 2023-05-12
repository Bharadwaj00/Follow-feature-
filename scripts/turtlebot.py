#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import copy
import math
import time
from geometry_msgs.msg import Twist


class TrackerNode:
    def __init__(self):
        self.tracker = cv2.TrackerBoosting_create()
        self.cv_image = None
        rospy.Subscriber("/robot2/camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/robot2/scan", LaserScan, self.laser_callback)
        # Create a publisher which can "talk" to Turtlesim and tell it to move
        self.vel_pub = rospy.Publisher('/robot2/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.laser_ranges = []
        self.laser_angles = []

        

    def PID(self,Kp, Ki, Kd, MV_bar=0):
        # initialize stored data
        e_prev = 0
        t_prev = -100
        I = 0
        
        # initial control
        MV = MV_bar
        
        while True:
            # yield MV, wait for new t, PV, SP
            t, PV, SP = yield MV
            
            # PID calculations
            e = SP - PV
            
            P = Kp*e
            I = I + Ki*e*(t - t_prev)
            D = Kd*(e - e_prev)/(t - t_prev)
            
            MV = MV_bar + P + I + D
            
            # update stored data for next iteration
            e_prev = e
            t_prev = t




    def image_callback(self, data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # image = cv2.flip(image, 1)
        self.cv_image = copy.deepcopy(image)
        # if self.cv_image is not None:
        #     print("Image received in callback function")
        # cv2.imshow("Image Window", self.cv_image)
        


    def laser_callback(self, data):
        self.laser_ranges = data.ranges
        self.laser_angles = [(data.angle_min + i * data.angle_increment) for i in range(len(data.ranges))]
        # self.laser_angles = data.angles

    def run(self):
    

        angular_vel_controller = self.PID(0.005, 0.0, 0.0)
        linear_vel_controller = self.PID(0.05, 0.0, 0.0)
        angular_vel_controller.send(None)
        linear_vel_controller.send(None)
        start_time = time.time()

        if not rospy.is_shutdown():
            frame = self.cv_image
            # Select ROI and initialize tracker
            flag_exit = False

            while self.cv_image is None:
                # if self.cv_image is not None:
                #     print("Frame has some value")
                print("Image not received yet")
                # key = input('Enter E to exit')
                # if(key == 'e'):
                #     flag_exit = True
                #     break
                # else:
                time.sleep(1)
                
                
            # cv2.imshow("Image Window", frame)
            bbox = cv2.selectROI("Tracking", self.cv_image, False)
            self.tracker.init(self.cv_image, bbox)


            while not rospy.is_shutdown() and not flag_exit: 
                frame = copy.deepcopy(self.cv_image)
                
                # Track object in current frame
                success, bbox = self.tracker.update(self.cv_image)
                current_time = time.time()
                if success:
                    frame_h, frame_w, _ = self.cv_image.shape
                    frame_cy = int(frame_h/2)
                    frame_cx = int(frame_w/2)
                    x, y, w, h = [int(i) for i in bbox]
                    cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Calculate center of bounding box
                    cx, cy = int(x + w/2), int(y + h/2)

                    if abs(cx-frame_cx) > 10:

                    
                        # next(angular_vel_controller)
                        angular_vel = angular_vel_controller.send([start_time - current_time, cx, frame_cx])

                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0
                        cmd_vel.angular.z = angular_vel
                        self.vel_pub.publish(cmd_vel)
                        print("Angular Velocity: {}".format(angular_vel))

                        

                    else:
                        # Convert center coordinates to cartesian and polar coordinates
                        # cartesian_coords = [cx, cy]
                        # polar_coords = [math.atan2(cy, cx), math.sqrt(cx**2 + cy**2)]

                        # Match polar coordinates with laser scan ranges using index
                        if len(self.laser_angles)>0:
                            # idx = min(range(len(self.laser_angles)), key=lambda i: abs(self.laser_angles[i]-polar_coords[0]))
                            # range_at_polar = self.laser_ranges[idx]
                            range_at_polar = self.laser_ranges[(len(self.laser_ranges)-1)/2]
                            print("Range at Polar Coordinates: {}".format(range_at_polar))
                            if math.isnan(range_at_polar):
                                range_at_polar = 5
                            
                            # next(linear_vel_controller)
                            linear_vel = linear_vel_controller.send([start_time - current_time, 1, range_at_polar])
                            cmd_vel = Twist()
                            cmd_vel.linear.x = linear_vel
                            cmd_vel.angular.z = 0
                            self.vel_pub.publish(cmd_vel)
                            
                            # Print cartesian and polar coordinates, as well as the range at the polar coordinates
                            # print("Cartesian Coordinates: ({}, {})".format(cx, cy))
                            # print("Polar Coordinates: ({}, {})".format(polar_coords[0], polar_coords[1]))
                            
                            print("Linear Velocity: {}".format(linear_vel))
                            
                           

                        else:
                            cmd_vel = Twist()
                            cmd_vel.linear.x = 0
                            cmd_vel.angular.z = 0
                            self.vel_pub.publish(cmd_vel)
                            print("Waiting for laser scan data...")

                else:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0
                    cmd_vel.angular.z = 0
                    self.vel_pub.publish(cmd_vel)
                    cv2.putText(self.cv_image, "Tracking Failed", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                    
                cv2.imshow("Image Window", self.cv_image)
                cv2.waitKey(1)

                start_time = current_time
    

    def cleanup_function(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('tracker_node', anonymous=True)
    node = TrackerNode()
    rospy.on_shutdown(node.cleanup_function)
    node.run()


