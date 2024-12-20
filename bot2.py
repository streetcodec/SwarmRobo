import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import time
import math
class Publisher(Node):
    def __init__(self):
        super().__init__('Bot2')
       # Initialze Publisher with the "/Integer" topic
        self.aruco = self.create_subscription(
            Pose2D, "/detected_aruco_2", self.PoseCB, 10
        )
        self.publisher_ = self.create_publisher(Int32, '/Vel2', 10)
        
        self.publisher_penpose1 = self.create_publisher(Int32, '/pen2_pose', 10)
        self.bot_1_x = [300,400,300,400]
        self.bot_1_y = [100,100,200,100]
        self.curr_x = 0
        self.curr_y = 0
        self.dist_x = 0
        self.dist_y = 0
        self.timer = self.create_timer(0.5,self.timer_callback)
        self.i = 1
        self.rev = 1000
        self.c = 0
        self.down = 1
        self.up = 0
        self.error_x = 0
        self.error_y = 0
        self.itr = 0
        self.leftVel = 0
        self.rightVel = 0
    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        msgPen1 = Int32()
        
        msgPen1.data = self.down
        self.error_x = self.bot_1_x[self.itr] - self.curr_x
        self.error_y = self.bot_1_y[self.itr] - self.curr_y
        if self.error_x < 1 and self.error_y < 1 :
            print(self.itr)
            self.itr += 1
            if self.itr > 4 :
                self.itr = 0

        self.p1 = 1
        self.i = 1
        msgPen1.data = self.down
        self.leftVel = ((-self.error_x / 2) - (math.cos((30*math.pi)/180)) * self.error_y) -0.05
        self.rightVel = ((-self.error_x / 2) + (math.cos((30*math.pi)/180)) * self.error_y) -0.05
        self.rearVel = 0.7*self.error_x -0.05
        i_str = str(self.i)
        if self.leftVel > 0 :
            i_str += "0"
        elif self.leftVel < 0 :
            i_str += "2" 
        elif self.leftVel == 0 :
            i_str += "1"
        
        if self.rightVel > 0 :
            i_str += "0"
        elif self.rightVel < 0 :
            i_str += "2" 
        elif self.rightVel == 0 :
            i_str += "1"

        if self.rearVel > 0 :
            i_str += "0"
        elif self.rearVel < 0 :
            i_str += "2" 
        elif self.rearVel == 0 :
            i_str += "1"
        
        self.i = int(i_str)
        msg.data = self.i





        self.publisher_.publish(msg) 
        self.get_logger().info('Publishing: %d' % msg.data)
        
        self.publisher_penpose1.publish(msgPen1)
        self.get_logger().info('Publishing PenPose2: %d' % msgPen1.data)
        time.sleep(0.5)  
        self.c = 1
    def PoseCB(self, msg):
         self.curr_x = msg.x
         self.curr_y = msg.y
         print(self.curr_x)
         print(self.curr_y)
         
def main(args = None):
    rclpy.init(args=args)
    Integer = Publisher()
    rclpy.spin(Integer)
    Integer.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()

# def timer_callback(self):
#         msg = Int32()
#         msg.data = self.i
#         msgPen1 = Int32()
        
#         msgPen1.data = self.down
#         if self.c == 1:
            
        
             
#             start_time2 = time.time()
#             self.c = 0
#             msg.data = self.rev
#             msgPen1.data = self.up
#             while time.time() - start_time2 < 1.2:
             
#              self.publisher_.publish(msg) 
#              self.get_logger().info('Publishing: %d' % msg.data)
            
#              self.publisher_penpose1.publish(msgPen1) 
#              self.get_logger().info('Publishing PenPose1: %d' % msgPen1.data)
            
#              time.sleep(0.5)
        
        
           
            
#         start_time = time.time()
        
#         while time.time() - start_time < 5:
#          self.p1 = 1
#          msg.data = self.i
#          msgPen1.data = self.down
#          self.publisher_.publish(msg) 
#          self.get_logger().info('Publishing: %d' % msg.data)
        
#          self.publisher_penpose1.publish(msgPen1)
#          self.get_logger().info('Publishing PenPose1: %d' % msgPen1.data)
#          time.sleep(0.5)  
#          self.c = 1


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import time

# class Publisher(Node):
#     def __init__(self):
#         super().__init__('Bot2')
#        # Initialze Publisher with the "/Integer" topic
#         self.publisher_ = self.create_publisher(Int32, '/Vel2', 10)
        
#         self.publisher_penpose1 = self.create_publisher(Int32, '/pen2_pose', 10)
        

#         self.timer = self.create_timer(0.5,self.timer_callback)
#         self.i = 1000
#         self.rev = 1000
#         self.c = 0
#         self.down = 0
#         self.up = 1
        
    
#     def timer_callback(self):
#         msg = Int32()
#         msg.data = self.i
#         msgPen1 = Int32()
        
#         msgPen1.data = self.down
        
#         if self.c == 1:
            
        
             
#             start_time2 = time.time()
#             self.c = 0
#             msg.data = self.rev
#             msgPen1.data = self.up
#             while time.time() - start_time2 < 1.2:
             
#              self.publisher_.publish(msg) 
#              self.get_logger().info('Publishing: %d' % msg.data)
            
#              self.publisher_penpose1.publish(msgPen1) 
#              self.get_logger().info('Publishing PenPose2: %d' % msgPen1.data)
            
#              time.sleep(0.5)
        
        
           
            
#         start_time = time.time()
        
#         while time.time() - start_time < 5:
#          self.p1 = 1
#          msg.data = self.i
#          msgPen1.data = self.down
#          self.publisher_.publish(msg) 
#          self.get_logger().info('Publishing: %d' % msg.data)
        
#          self.publisher_penpose1.publish(msgPen1)
#          self.get_logger().info('Publishing PenPose2: %d' % msgPen1.data)
#          time.sleep(0.5)  
#          self.c = 1
# def main(args = None):
#     rclpy.init(args=args)
#     Integer = Publisher()
#     rclpy.spin(Integer)
#     Integer.destroy_node()
#     rclpy.shutdown()
 
# if __name__ == '__main__':
#     main()