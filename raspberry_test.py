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


old_sec = 0
new_sec = 0

pixel_yatay = 640
pixel_dikey = 480

#---yaw_pid ------
pre_hata_yaw_pix = 0
hata_yaw_pix = 0

def track_yaw(curr_yaw,target_yaw_pix):
        global pre_hata_yaw_pix 
        global hata_yaw_pix
        p_yaw = 0.2
        d_yaw = 0.01
        hata_yaw_pix = 320 - target_yaw_pix
        print(hata_yaw_pix)
        if (hata_yaw_pix <= 0):#saat yonu
        #yaw hareketi saat yonunde olacaktir        
                calc_yaw =  ((p_yaw * hata_yaw_pix) + (d_yaw *(hata_yaw_pix - pre_hata_yaw_pix)))
                publish_yaw(calc_yaw)
                pre_hata_yaw_pix = hata_yaw_pix
                print("calc_yaw ",calc_yaw)
                
               
        if (hata_yaw_pix > 0): #saat yonu tersi
                calc_yaw =((p_yaw * hata_yaw_pix) + (d_yaw *(hata_yaw_pix - pre_hata_yaw_pix)))
                publish_yaw(calc_yaw)
                pre_hata_yaw_pix = hata_yaw_pix
                print("calc_yaw",calc_yaw)




def publish_yaw(new_yaw):
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    data = TwistStamped()
    #print(data)
    
    data.twist.angular.x = 0
    data.twist.angular.y = 0
    data.twist.angular.z = math.radians(new_yaw)
    pub.publish(data)    
    #print(data)
    #rospy.loginfo(data)


def target_line_on_screen(image):

        #cv2.line(image,((pixel_yatay/2)-10,pixel_dikey/2),((pixel_yatay/2)-30,pixel_dikey/2),(150,255,50),2)
        #cv2.line(image,((pixel_yatay/2)+10,pixel_dikey/2),((pixel_yatay/2)+30,pixel_dikey/2),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 + 10,(pixel_dikey/2)-10),(pixel_yatay/2+10,(pixel_dikey/2)-30),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 + 10,(pixel_dikey/2)+10),(pixel_yatay/2+10,(pixel_dikey/2)+30),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 - 10,(pixel_dikey/2)-10),(pixel_yatay/2-10,(pixel_dikey/2)-30),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 - 10,(pixel_dikey/2)+10),(pixel_yatay/2-10,(pixel_dikey/2)+30),(150,255,50),2)

        cv2.line(image,(0,240),(640,240),(150,255,50),2)

def callback(msg):
        global new_sec
        global old_sec
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        new_sec = msg.header.seq
        target_line_on_screen(cv_image)
        if (new_sec > old_sec):
                old_sec = new_sec
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(gray_image,235,255,cv2.THRESH_BINARY)

                cnts=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
                center=None
	        if len(cnts)>0:
                        c=max(cnts,key=cv2.contourArea)
                        ((x,y),radius)=cv2.minEnclosingCircle(c)
                                              
                        #cv2.circle(cv_image,(int(x),int(y)),int(radius),(0,255,0))
                        track_yaw(yaw_degree,x)

                cv2.imshow("frame", cv_image)
        

       

       
        if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("finished.")



def image_sub():
        rospy.init_node('cv_stream', anonymous=False)
        sub = rospy.Subscriber('line', Image, callback)
        #rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)
        
        rospy.spin()
	
        cv2.destoryAllWindows()

if __name__ == '__main__':
        image_sub()
       