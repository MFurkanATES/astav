import rospy
from std_msgs.msg import String as abc 

import pickle

def sub_data(data): 
  
  #yeni_data = r[r.find("[") +1 : r.find("]")].split(" ")
  data = data.data
  data = [int(i.split(".")[0]) for i in data.replace("["," ").replace("]"," ").split()]
  print(data[0])



def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('/chatter',abc,sub_data)
  
  
  
while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()
