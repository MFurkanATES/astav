import rospy
import numpy as np
import cv2
import math
import tf
import kalman_filter as kalman
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import *
from sensor_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


pixel_yatay = 720
pixel_dikey = 480

X = 0 
Y = 0
Z = 0

#---stats---

mode = 'null'
armed_status = 'null'
voltage = 0.0
current = 0.0
percentage = 0.0
sat_num = 0
latitude = 0.0
longitude = 0.0 
service = 9
gps_fix_status = 9
relative_altitude = 0.0
heading_compass = 0
air_speed = 0.0
ground_speed = 0.0
throttle = 0.0


#---track---
track_flag = 0
track_status = "-----"
pilot_conf = 0
blind_track = 0
obj_data = 0

#---yaw_pid ---
pre_hata_yaw_pix = 0
hata_yaw_pix = 0

#---roll_pid ---
pre_hata_roll_pix = 0
hata_roll_pix = 0

#test

flag_pub_yaw = 0
flag_pub_roll = 0



def set_stream_rate():
	
  print("stream ayarlaniyor")
  rospy.wait_for_service('/mavros/set_stream_rate')
  try:
    stream_service = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
    stream_response = stream_service(0,30,1)
    rospy.loginfo(stream_response)
  except rospy.ServiceException as e:
    print("set_stream servisi hatayla karsilasti: %s" %e)
#--------------------------------------
def track_data_line(data):
  
def track(image):

  if track_flag == 1 and obj_data == 1:
    pass
    
  
  
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

#--------------------------------------
def autopilot_rcin_status(rc):
  global pilot_conf
  global track_status
  global track_flag 

  pilot_conf =  rc.channels[6]
  if pilot_conf > 1500:
    track_status = "-----"
    track_flag = 0
  if pilot_conf < 1500:
    track_status = "TRACK"
    track_flag = 1

def autopilot_state(state):
  global armed_status
  global mode
  mode = state.mode
  if state.armed == False:
    armed_status = 'DISARMED'
  else:
    armed_status = 'ARMED'
  #print (armed_status,mode)
  #otopilotun arm-disarm durumunu verir
  
def autopilot_battery_status(batt_state):
  global voltage 
  global current
  global percentage
  voltage = round(batt_state.voltage,2)
  current = round(batt_state.current,2) 
  percentage = round((batt_state.percentage*100),2)
  #print (voltage,current,percentage)
  #otopilotun pil degerlerini verir
  
def autopilot_gps_sat(sat_status):
  global sat_num 
  sat_num = sat_status.data
  #print sat
  #otopilotun kac uydu baglantisinda oldugunu verir
  
def autopilot_gps_status(gps_status):
  global latitude 
  global longitude 
  global service
  global gps_fix_status
  latitude = round(gps_status.latitude,7)
  longitude = round(gps_status.longitude,7) 
  service = gps_status.status.service
  gps_fix_status = gps_status.status.status
  #print latitude,longitude,service,gps_fix_status
  #otopilotun gpsinin lat,lon ve durum bilgilerini verir
  
def autopilot_hud_status(hud_status):
  global heading_compass 
  global air_speed 
  global ground_speed 
  global throttle
  heading_compass = hud_status.heading 
  air_speed = round(hud_status.airspeed,2)
  ground_speed = round(hud_status.groundspeed,2)
  throttle = round((hud_status.throttle*100),2)
  #print heading_compass,air_speed,ground_speed,throttle
  #print (hud_status.heading)
  
def autopilot_imu_orientation(imu_orientation):
  
  global X
  global Y
  global Z
  #x = imu_orientation.orientation.x
  #y = imu_orientation.orientation.y
  #z = imu_orientation.orientation.z
  #w = imu_orientation.orientation.w
  quaternion =(imu_orientation.orientation.x,imu_orientation.orientation.y,imu_orientation.orientation.z,imu_orientation.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  #roll 
  
  X = euler[0]
  Y = euler[1]
  Z = euler[2]

  
def callback(msg):
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
  #print("seq",msg.header.seq)
  info_on_screen(cv_image)
  cv2.imshow("frame", cv_image)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    rospy.loginfo("finished.")
  
  
#------------------------------------------
def target_line_on_screen(image):
  global pixel_yatay
  global pixel_dikey

  cv2.line(image,(int((pixel_yatay/2))-10,int(pixel_dikey/2)),(int(pixel_yatay/2)-30,int(pixel_dikey/2)),(150,255,50),2)
  cv2.line(image,(int(pixel_yatay/2)+10,int(pixel_dikey/2)),(int(pixel_yatay/2)+30,int(pixel_dikey/2)),(150,255,50),2)
  cv2.line(image,(int(pixel_yatay/2),int(pixel_dikey/2)-10),(int(pixel_yatay/2),int(pixel_dikey/2)-30),(150,255,50),2)
  cv2.line(image,(int(pixel_yatay/2),int(pixel_dikey/2)+10),(int(pixel_yatay/2),int(pixel_dikey/2)+30),(150,255,50),2)
  

def hedef_cizgisi(image):
  global pixel_yatay
  global pixel_dikey
  cv2.rectangle(image,(int(pixel_yatay/4),int(pixel_dikey/10)),(int(3 * pixel_yatay/4),int(9 * pixel_dikey/10)),(150,255,50),1)


def pitch_line_on_screen(image):
  #yan cizgiler
  cv2.line(image,(210,160),(210,320),(150,255,50),1)
  cv2.line(image,(430,160),(430,320),(150,255,50),1)
  #orta cizgiler+ (math.pi/2)
  cv2.line(image,(210,240),(220,240),(150,255,50),1)
  cv2.line(image,(420,240),(430,240),(150,255,50),1)
  #uc cizgiler
  cv2.line(image,(190,320),(210,320),(150,255,50),1)
  cv2.line(image,(190,160),(210,160),(150,255,50),1)
  cv2.line(image,(430,320),(450,320),(150,255,50),1)
  cv2.line(image,(430,160),(450,160),(150,255,50),1)
  #cv2.putText(image,str(0),(195,245 ),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  #cv2.putText(image,str(0),(415,245 ),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

  
def pitch_on_screen(image):
  global Y
  global pixel_yatay
  global pixel_dikey
  screen_angle_pitch = -1 * int(math.degrees(Y))
  #print screen_angle_pitch
  set_pixel = (screen_angle_pitch * 2)  
  cv2.line(image,(440,(240 + set_pixel)),(430,(240 + set_pixel)),(150,255,50),2)
  cv2.line(image,(200,(240 + set_pixel)),(210,(240 + set_pixel)),(150,255,50),2)
  cv2.putText(image,str(screen_angle_pitch),(440,240 + set_pixel),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,str(screen_angle_pitch),(180,240 + set_pixel),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  
  
def status_on_screen(image):
  global mode
  global armed_status

  cv2.putText(image,("MOD:" + str(mode)),(5,20),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("IZIN:" + str(armed_status)),(5,35),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  
def batt_on_screen(image):
  global voltage
  global percentage
  global current

  cv2.putText(image,("VOLTAJ:" + str(voltage)),(5,170),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("AKIM:"   + str(current)),(5,185),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("PIL:%"   + str(percentage)),(5,200),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def sat_on_screen(image):
  global sat_num
  global longitude
  global latitude

  cv2.putText(image,("GPS UYDU:" + str(sat_num)),(5,50),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("LAT:"     + str(latitude)),(5,65),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("LON:"     + str(longitude)),(5,80),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  
def hud_on_screen(image):
  global air_speed
  global ground_speed
  global throttle
  global heading_compass
  global relative_altitude

  cv2.putText(image,("YER HIZI:"  + str(ground_speed)),(5,95),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("HAVA HIZI:" + str(air_speed)),(5,110),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("GAZ:%"      + str(throttle)),(5,125),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("PUSULA:"    + str(heading_compass)),(5,140),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("YUKSEKLIK:" + str(relative_altitude)),(5,155),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  
def track_stats(image):
  
  global track_status
  cv2.putText(image,(track_status),(pixel_yatay-100,20),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255),2)
  
#------------------------------------------
def info_on_screen(image):
  status_on_screen(image)
  batt_on_screen(image)
  sat_on_screen(image)
  hud_on_screen(image)
  target_line_on_screen(image)
  pitch_line_on_screen(image)
  pitch_on_screen(image)
  hedef_cizgisi(image)
  track_stats(image)
  
def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/state',State,autopilot_state)
  rospy.Subscriber('mavros/battery',BatteryState,autopilot_battery_status)
  rospy.Subscriber('mavros/global_position/global',NavSatFix,autopilot_gps_status)
  rospy.Subscriber('mavros/global_position/raw/satellites',UInt32,autopilot_gps_sat)
  rospy.Subscriber('mavros/vfr_hud',VFR_HUD,autopilot_hud_status)
  rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)
  rospy.Subscriber('mavros/rc/in',RCIn,autopilot_rcin_status)
  rospy.Subscriber('chatter', String,track_data_line)
  sub = rospy.Subscriber('line', Image, callback)
set_stream_rate()
while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()
