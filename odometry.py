import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray                                                                                                
import math
class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_calculator')
        
        self.last_time = rospy.Time.now()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.velocity_left = 0.0
        self.velocity_right = 0.0
        self.angular= 0.0

        self.wheel_radius = 0.135  
        
        self.imu_sub = rospy.Subscriber('/imu1/data', Imu, self.imu_callback)
        self.left_encoder_sub = rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, self.left_encoder_callback)
        self.right_encoder_sub = rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, self.right_encoder_callback)
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # converting quaternion to euler 
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_theta = yaw
        self.angular= msg.angular_velocity.z

    def left_encoder_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        # Converting left motor rpm data to velocity
        self.velocity_left = msg.data[0]        
        self.velocity_left = 2 * math.pi * self.wheel_radius * (self.velocity_left / 60)
        self.current_x += self.velocity_left * dt * cos(self.current_theta)
        self.current_y += self.velocity_left * dt * sin(self.current_theta)      # Setting the location points on x and y axis'
        self.last_time = current_time  
        self.publish_odometry()

    def right_encoder_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        # Converting right motor rpm data to velocity
        self.velocity_right = msg.data[0]
        
        self.velocity_right= 2 * math.pi * self.wheel_radius * (self.velocity_right / 60)

        self.current_x += self.velocity_right * dt * cos(self.current_theta)
        self.current_y += self.velocity_right * dt * sin(self.current_theta)    # Setting the location points on x and y axis'
        self.last_time = current_time
        self.publish_odometry()

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set the position
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0

        # Convert the Euler angles to quaternion
        quaternion = quaternion_from_euler(0, 0, self.current_theta)
        odom.pose.pose.orientation = Quaternion(*quaternion)

        # Set the velocity
        odom.twist.twist.linear.x = (self.velocity_left + self.velocity_right) / 2.0
        odom.twist.twist.linear.y = (self.velocity_left + self.velocity_right) / 2.0      
        odom.twist.twist.angular.z = self.angular
        self.odom_pub.publish(odom)

    def spin(self):
        rate = rospy.Rate(10)  
        
        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odom_calc = OdometryCalculator()
        odom_calc.spin()
    except rospy.ROSInterruptException:
        pass
