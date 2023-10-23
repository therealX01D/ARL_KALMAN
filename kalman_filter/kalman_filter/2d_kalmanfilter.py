import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from math import cos,sin
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        self.dt = 0.035  # Time step
        self.A = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # State transition matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # Measurement matrix
        self.Q = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [self.dt, 0, self.dt**2, 0],
                           [0, self.dt, 0, self.dt**2]])  # Process noise covariance matrix
        self.R = np.array([[0.1, 0],
                           [0, 0.1]])  # Measurement noise covariance matrix
        self.state = np.zeros((4, 1))  # Initial state
        self.covariance = np.eye(4)
        # self.x=np.zeros((2,1))
        # self.P=np.eye(2)
        # # Initialize kalman variables
        # self.position=np.zeros((3,1))
        # self.nextposition=np.zeros((3,1))
        # self.veli= np.array([[0.99],[0.99],[0.0]])
        # self.cov = np.eye(3)
        # self.dt = 0.035
        # self.rn = 0.01
        # self.prevpos=np.zeros((3,1))
        # self.elevpos =np.zeros((3,1))
        # Subscribe to the /odom_noise topic
        self.subscription2 = self.create_subscription(Twist,
                                                      '/odom',
                                                      self.odom_callback2,
                                                      1)
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)

        #publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)
    def odom_callback2(self, msg):
        # Extract the position measurements from the Odometry message

        self.veli=np.array([[msg.linear.x]
         ,[msg.linear.y]
         ,[msg.linear.z]])
        yaw = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.A[0, 2] = self.dt * self.veli[0,0]*cos(yaw)
        self.A[1, 3] = self.dt * self.veli[0,0]*sin(yaw)

    def odom_callback(self, msg):
        measurements=np.array([[msg.pose.pose.position.x] ,[msg.pose.pose.position.y]])
        self.state = self.A @ self.state
        self.covariance = self.A @ self.covariance @ self.A.T 
        y = measurements - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.covariance = (np.eye(4) - K @ self.H) @ self.covariance
        new_pos=self.state[:2, 0]
        estimated_msg = Odometry()
        estimated_msg.pose.pose.position.x = new_pos[0]
        #self.position[0,0]
        estimated_msg.pose.pose.position.y = new_pos[1]
        self.estimated_pub.publish(estimated_msg)
        # # Extract the position measurements from the Odometry message
        # self.elevpos=np.copy(self.nextposition)
        # reci=np.array([[msg.pose.pose.position.x]
        # ,[msg.pose.pose.position.y]
        # ,[msg.pose.pose.position.z]])


        # # Prediction step
        # self.nextposition=self.nextposition + self.veli *self.dt
        # K = self.cov/(self.cov + self.rn)
        # # Update step
        # self.position = self.elevpos + K*(reci - self.elevpos)
        # self.cov = (1-K)*self.cov
        # self.get_logger().info("k is "+ str(K))

        # #publish the estimated reading
        # estimated_msg = Odometry()
        # estimated_msg.pose.pose.position.x = self.position[0,0]
        # #self.position[0,0]
        # estimated_msg.pose.pose.position.y = self.position[1,0]
        # estimated_msg.pose.pose.position.z = self.position[2,0]
        # self.estimated_pub.publish(estimated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
