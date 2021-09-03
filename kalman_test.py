import cv2
import math
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped,Vector3,TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import PositionTarget



#--vision-----
window_name2 = "Hue histogram back projection" 

old_sec = 0
new_sec = 0


pixel_yatay = int(640)
pixel_dikey = int(480)


#---orientation-----
X = 0.0
Y = 0.0
Z = 0.0
roll_degree = 0.0
pitch_degree = 0.0
yaw_degree = 0.0
quaternion_X = 0.0
quaternion_Y = 0.0
quaternion_Z = 0.0
quaternion_W = 0.0


#---yaw_pid ------
pre_hata_yaw_pix = 0
hata_yaw_pix = 0

#---roll_pid ------
pre_hata_roll_pix = 0
hata_roll_pix = 0

#test
global flag_pub_yaw 
global flag_pub_rol

flag_pub_yaw = 0
flag_pub_roll = 0





def track_yaw(curr_yaw,target_yaw_pix):
        global pre_hata_yaw_pix 
        global hata_yaw_pix
        p_yaw = 0.29
        d_yaw = 0.01
        hata_yaw_pix = 320 - target_yaw_pix
        #print(hata_yaw_pix)
        if (hata_yaw_pix <= 0):#saat yonu
        #yaw hareketi saat yonunde olacaktir        
                calc_yaw =  ((p_yaw * hata_yaw_pix) + (d_yaw *(hata_yaw_pix - pre_hata_yaw_pix)))
                publish_yaw(calc_yaw)
                pre_hata_yaw_pix = hata_yaw_pix
                #print("calc_yaw ",calc_yaw)
                
               
        if (hata_yaw_pix > 0): #saat yonu tersi
                calc_yaw =((p_yaw * hata_yaw_pix) + (d_yaw *(hata_yaw_pix - pre_hata_yaw_pix)))
                publish_yaw(calc_yaw)
                pre_hata_yaw_pix = hata_yaw_pix
                #print("calc_yaw",calc_yaw)
        rospy.loginfo("yaw out: %f",calc_yaw)



def track_roll(curr_roll,target_roll_pix):
        global pre_hata_roll_pix 
        global hata_roll_pix
        p_roll = 0.002
        d_roll = 0.0001
        hata_roll_pix = 240 - target_roll_pix
        #print(hata_roll_pix)
        if (hata_roll_pix <= 0):#saat yonu
        #yaw hareketi saat yonunde olacaktir        
                calc_roll =  ((p_roll * hata_roll_pix) + (d_roll *(hata_roll_pix - pre_hata_roll_pix)))
                publish_roll(calc_roll)
                pre_hata_roll_pix = hata_roll_pix
                #print("calc_roll ",calc_roll)
                
               
        if (hata_roll_pix > 0): #saat yonu tersi
                calc_roll =((p_roll * hata_roll_pix) + (d_roll *(hata_roll_pix - pre_hata_roll_pix)))
                publish_roll(calc_roll)
                pre_hata_roll_pix = hata_roll_pix
                #print("calc_roll",calc_roll)
        rospy.loginfo("roll out: %f",calc_roll)




def publish_yaw(new_yaw):
    global flag_pub_yaw 
    global flag_pub_roll  
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    data = TwistStamped()
    data.twist.angular.z = math.radians(new_yaw)
    if (flag_pub_yaw == 0):
    
    	pub.publish(data) 
    	flag_pub_roll = 0
    	flag_pub_yaw  = 1 
 
   
     
   
  
   

def publish_roll(new_roll):
    global flag_pub_yaw 
    global flag_pub_roll

    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    data = TwistStamped()    
    data.twist.linear.z = new_roll
    #data.twist.angular.x = math.radians(new_roll)  
    if (flag_pub_roll == 0):
    
    	pub.publish(data) 
    	flag_pub_roll = 1
    	flag_pub_yaw = 0
  
   


def target_line_on_screen(image):

        #cv2.line(image,((pixel_yatay/2)-10,pixel_dikey/2),((pixel_yatay/2)-30,pixel_dikey/2),(150,255,50),2)
        #cv2.line(image,((pixel_yatay/2)+10,pixel_dikey/2),((pixel_yatay/2)+30,pixel_dikey/2),(150,255,50),2)
        cv2.line(image,(330,230),(330,210),(150,255,50),2)
        cv2.line(image,(330,250),(330,270),(150,255,50),2)
        cv2.line(image,(310,230),(310,210),(150,255,50),2)
        cv2.line(image,(310,250),(310,270),(150,255,50),2)
        cv2.line(image,(0,240),(640,240),(150,255,50),2)

def callback(msg):
        global new_sec
        global old_sec
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        new_sec = msg.header.seq
        
        target_line_on_screen(cv_image)
        #if (new_sec > old_sec):
                
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray_image,245,255,cv2.THRESH_BINARY)

        cnts=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center=None
        if len(cnts)>0:
                c=max(cnts,key=cv2.contourArea)
                ((x,y),radius)=cv2.minEnclosingCircle(c)
                cv2.line(cv_image,(int(320),int(240)),(int(x),int(y)),(150,255,50),1)                  
                #cv2.circle(cv_image,(int(x),int(y)),int(radius),(0,255,0))
                track_yaw(yaw_degree,x)
                rospy.sleep(0.04)                
                track_roll(roll_degree,y)
                
            
        cv2.imshow("frame", cv_image)
	

        	#old_sec = new_sec

       

       
        if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("finished.")




def autopilot_imu_orientation(imu_orientation):  
        global X
        global Y
        global Z
        global roll_degree
        global pitch_degree
        global yaw_degree
        global quaternion_X
        global quaternion_Y
        global quaternion_Z
        global quaternion_W

        quaternion_X = imu_orientation.orientation.x
        quaternion_Y = imu_orientation.orientation.y
        quaternion_Z = imu_orientation.orientation.z
        quaternion_W = imu_orientation.orientation.w

        quaternion =(imu_orientation.orientation.x,imu_orientation.orientation.y,imu_orientation.orientation.z,imu_orientation.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        #radian
        X = euler[0]
        Y = euler[1]
        Z = euler[2]
        #degree
        roll_degree = int(abs(math.degrees(X)))
        pitch_degree = int(abs(math.degrees(Y)))
        yaw_degree = int(abs(math.degrees(Z)))




def image_sub():
        rospy.init_node('cv_stream', anonymous=False)
        sub = rospy.Subscriber('line', Image, callback)
        #rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)
        
        rospy.spin()
	
        cv2.destoryAllWindows()

if __name__ == '__main__':
        image_sub()
       
