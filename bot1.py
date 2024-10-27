import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import time
import math
class Publisher(Node):
    def __init__(self):
        super().__init__('Bot1')
       # Initialze Publisher with the "/Integer" topic
        self.aruco = self.create_subscription(
            Pose2D, "/detected_aruco_1", self.PoseCB, 10
        )
        self.arucoAngle = self.create_subscription(Float32 , "/AngleAruco1", self.GetAngle , 10)
        self.publisherLeft = self.create_publisher(Int32, '/leftVel1', 10)
        self.publisherRight = self.create_publisher(Int32, '/rightVel1', 10)
        self.publisherRear = self.create_publisher(Int32, '/rearVel1', 10)
        
        self.publisher_penpose1 = self.create_publisher(Int32, '/pen1_pose', 10)
        self.bot_1_x = [200,175,125,100,125,175,200]
        self.bot_1_y = [150,200,200,150,100,100,150]
        # self.bot_1_x = [250,250,250,250,250,250,250]
        # self.bot_1_y = [250,250,250,250,250,250,250]
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
        self.rearVel = 0
        self.theta = 0
        self.kp = 1
    def timer_callback(self):
        msgLeft = Int32()
        msgRight = Int32()
        msgRear = Int32()

        #msg.data = self.i
        msgPen1 = Int32()
        
        msgPen1.data = self.down
        # self.error_x = self.bot_1_x[self.itr] - self.curr_x
        # self.error_y = self.bot_1_y[self.itr] - self.curr_y
        self.error_x = 250 - self.curr_x
        self.error_y = 250 - self.curr_y
        self.velo_x = self.kp * self.error_x
        self.velo_y = self.kp * self.error_y

        if abs(self.error_x) < 5 and abs(self.error_y) < 5 :
            print(self.itr)
            self.itr += 1
            iii = 0
            if self.itr > 6 :
                self.itr = 0
            while iii < 10 :
                self.leftVel = 90
                self.rightVel = 90
                self.rearVel = 90
                msgLeft.data = self.leftVel
                msgRight.data = self.rightVel
                msgRear.data = self.rearVel
                self.publisherLeft.publish(msgLeft) 
                self.get_logger().info('Publishing Left : %d' % msgLeft.data)
                self.publisherRight.publish(msgRight) 
                self.get_logger().info('Publishing Right : %d' % msgRight.data)
                self.publisherRear.publish(msgRear) 
                self.get_logger().info('Publishing Rear : %d' % msgRear.data)
                    
                self.publisher_penpose1.publish(msgPen1)
                self.get_logger().info('Publishing PenPose1: %d' % msgPen1.data)
                # print(self.curr_x)
                # print(self.curr_y)
                iii += 1
                time.sleep(0.5)
            
        msgPen1.data = self.up
        # self.p1 = 1
        # self.i = 1
        msgPen1.data = self.down
        self.leftVeel = ((-self.velo_x / 2) - (math.cos((30*math.pi)/180)) * self.velo_y) 
        self.rightVeel = ((-self.velo_x / 2) + (math.cos((30*math.pi)/180)) * self.velo_y)
        self.rearVeel = self.velo_x

        # self.leftVeel = -131 #rpm
        # self.rightVeel = 131 #rpm
        # self.rearVeel = 0 #rpm
        print("lw", self.leftVel)
        print("rw", self.rightVel)
        print("rew", self.rearVel)
        

        
        if self.leftVeel >= 0 :
            self.leftVel = 91 - pow(1.035,self.leftVeel)
        elif self.leftVeel < 0 :
            self.leftVel = 89 + pow(1.035,-self.leftVeel)
        
        if self.rightVeel >= 0 :
            self.rightVel = 91 - pow(1.035,self.rightVeel)
        elif self.rightVeel < 0 :
            self.rightVel = 89 + pow(1.035,-self.rightVeel)
        
        if self.rearVeel >= 0 :
            self.rearVel = 91 - pow(1.035,self.rearVeel)
        elif self.rearVeel < 0 :
            self.rearVel = 89 + pow(1.035,-self.rearVeel)
        

        print("lw", self.leftVel)
        print("rw", self.rightVel)
        print("rew", self.rearVel)

         


        # self.leftVel = math.exp(self.leftVel)%16000
        # self.rightVel = math.exp(self.rightVel)%16000
        # self.rearVel = math.exp(self.rearVel)%16000
        # if self.leftVel == 0:
        #     self.leftVel = 90
        # if self.rightVel == 0 :
        #     self.rightVel = 90
        # if self.rearVel == 0 :
        #     self.rearVel = 90
        
        # if self.leftVel < 2000:
        #     self.leftVel = 2000


        # if self.rightVel < 2000 :
        #     self.rightVel = 2000
        # if self.rearVel < 2000 :
        #     self.rearVel = 2000
        # left_pwm = int(self.scale(self.leftVel, min(self.leftVel, self.rightVel, self.rearVel), max(self.leftVel, self.rightVel, self.rearVel),2000,16000))
        # right_pwm = int(self.scale(self.rightVel, min(self.leftVel, self.rightVel, self.rearVel), max(self.leftVel, self.rightVel, self.rearVel),2000,16000))
        # rear_pwm = int(self.scale(self.rearVel, min(self.leftVel, self.rightVel, self.rearVel), max(self.leftVel, self.rightVel, self.rearVel),2000,16000))

        self.leftVel = int(self.leftVel)
        self.rightVel = int(self.rightVel)
        self.rearVel = int(self.rearVel)

        # self.leftVel = 90 # 33 pin
        # self.rightVel = 90 # 25 pin
        # self.rearVel = 90 # 26 pin

        msgLeft.data = self.leftVel
        msgRight.data = self.rightVel
        msgRear.data = self.rearVel

        # msgLeft.data = left_pwm
        # msgRight.data = right_pwm
        # msgRear.data = rear_pwm
        k = 0

        # for ii in range (2000,16000) :

        #     self.leftVel = 90 # 33 pin rear
        #     self.rightVel = 0 # 25 pin left
        #     self.rearVel = 90 # 26 pin right
        #     msgLeft.data = self.leftVel
        #     msgRight.data = self.rightVel
        #     msgRear.data = self.rearVel
        #     self.publisherLeft.publish(msgLeft) 
        #     self.get_logger().info('Publishing Left : %d' % msgLeft.data)
        #     self.publisherRight.publish(msgRight) 
        #     self.get_logger().info('Publishing Right : %d' % msgRight.data)
        #     self.publisherRear.publish(msgRear) 
        #     self.get_logger().info('Publishing Rear : %d' % msgRear.data)
                
        #     self.publisher_penpose1.publish(msgPen1)
        #     self.get_logger().info('Publishing PenPose1: %d' % msgPen1.data)
        #     ii += 1
        #     k += 5
            # if self.leftVel == 0 :
            #     self.leftVel = 90
            # elif self.leftVel == 90 :
            #     self.leftVel = 180
            # elif self.leftVel == 180 :
            #     self.leftVel = 0

            # if self.rightVel == 0 :
            #     self.rightVel = 90
            # elif self.rightVel == 90 :
            #     self.rightVel = 180
            # elif self.rightVel == 180 :
            #     self.rightVel = 0

            # if self.rearVel == 0 :
            #     self.rearVel = 90
            # elif self.rearVel == 90 :
            #     self.rearVel = 180
            # elif self.rearVel == 180 :
            #     self.rearVel = 0
                    
            #time.sleep(5)
        self.publisherLeft.publish(msgLeft) 
        self.get_logger().info('Publishing Left : %d' % msgLeft.data)
        self.publisherRight.publish(msgRight) 
        self.get_logger().info('Publishing Right : %d' % msgRight.data)
        self.publisherRear.publish(msgRear) 
        self.get_logger().info('Publishing Rear : %d' % msgRear.data)
        
        self.publisher_penpose1.publish(msgPen1)
        self.get_logger().info('Publishing PenPose1: %d' % msgPen1.data)
        # print(self.itr)
        # print(self.theta)
        
          
        # self.c = 1
    def PoseCB(self, msg):
         if msg.x == 1000 :
             print("Aruco 1 not detected")
             return
         self.curr_x = msg.x
         self.curr_y = msg.y# self.leftVel 
        #  print(self.curr_x)
        #  print(self.curr_y)
    def GetAngle(self , msg) :
        self.theta = msg.data
    # def scale(self , value, in_min, in_max, out_min, out_max):
    #      return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
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