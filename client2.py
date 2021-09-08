import requests
from enum import Enum
import os
import json
import time
import rospy
import math
import tf
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image,NavSatFix,BatteryState,TimeReference
from std_msgs.msg import Float64,String
from mavros_msgs.msg import VFR_HUD
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
import sys

endpoint = "http://192.168.20.10:64559"
cookie = None # "Buraya"
#cookie = ""
#384.satir logini kapat yeniden baslatirken

kilitlenme_merkez_x = 0
kilitlenme_merkez_y = 0
kilitlenme_bilgisi_genislik = 0
kilitlenme_bilgisi_dikey = 0
kilitlenme_bilgisi_sonuc = 0
saniye_eski = 0
genislik_eski = 0
ms_eski = 0




X = 0.0
Y = 0.0
Z = 0.0
X_degree = 0.0
Y_degree = 0.0
Z_degree = 0.0
quaternion_X = 0.0
quaternion_Y = 0.0
quaternion_Z = 0.0
quaternion_W = 0.0
roll = 0
pitch = 0

latitude = 0.0 
longitude = 0.0
altitude = 0.0
relative_altitude = 0.0


heading_compass = 0
air_speed = 0.0
ground_speed = 0.0
throttle = 0.0

voltage = 0.0
current = 0.0
percentage = 0.0
kilitlendi_saat = None
kilitlenme_bitti_saat = None

mode = 'null'
armed_status = 'null'
mode_status = 0
kilitlenme_eski_durum = 0

saat_data = 0
def autopilot_state(state):
  global armed_status
  global mode
  global mode_status
  mode = state.mode
  if state.armed == False:
    armed_status = 'DISARMED'
  else:
    armed_status = 'ARMED'

  if mode == "GUIDED" or mode == "AUTO" or mode == "RTL":
      mode_status = 1
  else: 
      mode_status = 0
  
  #print (armed_status,mode)
  #otopilotun arm-disarm durumunu verir

def autopilot_imu_orientation(imu_orientation):  
  global X
  global Y
  global Z
  global X_degree
  global Y_degree
  global Z_degree
  global quaternion_X
  global quaternion_Y
  global quaternion_Z
  global quaternion_W
  global roll
  global pitch

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
  #x roll angle
  X_degree = int(math.degrees(X))
  roll = X_degree
  #Y pitch angle
  Y_degree = int(math.degrees(Y))
  pitch = Y_degree
  Z_degree = int(math.degrees(Z))
  
  #print(X,Y,Z,X_degree,Y_degree,Z_degree,quaternion_X,quaternion_Y,quaternion_Z,quaternion_W)


def autopilot_gps_status(gps_status):
  global latitude 
  global longitude 
  global altitude
  global gps_service
  global gps_fix_status
  latitude = round(gps_status.latitude,7)
  longitude = round(gps_status.longitude,7) 
  altitude = round(gps_status.altitude,2)
  gps_service = gps_status.status.service
  gps_fix_status = gps_status.status.status
  #print (latitude,longitude,altitude,gps_service,gps_fix_status)
  #otopilotun gpsinin lat,lon,alt ve durum bilgilerini verir
  #alt bilgisi deniz seviyesine goredir 


def autopilot_rel_alt_status(alt_status):
  global relative_altitude
  relative_altitude = round(alt_status.data,1)
  #print (relative_altitude)
  #otopilotun bulunan irtifadan  itibaren yuksekligini verir

def autopilot_hud_status(hud_status):
  global heading_compass 
  global air_speed 
  global ground_speed 
  global throttle
  heading_compass = hud_status.heading 
  air_speed = round(hud_status.airspeed,2)
  ground_speed = round(hud_status.groundspeed,2)
  throttle = round((hud_status.throttle*100),2)
  #print (heading_compass,air_speed,ground_speed,throttle)



def autopilot_battery_status(batt_state):
  global voltage 
  global current
  global percentage
  voltage = round(batt_state.voltage,2)
  current = round(batt_state.current,2) 
  percentage = round((batt_state.percentage*100),2)
  #print (voltage,current,percentage)
  #otopilotun pil degerlerini verir

def saat(data):
  global saat_data   
 

  a = data.time_ref.secs 
  b = data.time_ref.nsecs
  saat_data = str(time.strftime('%H.%M.%S.{}'.format(str(b)[:3]), time.gmtime(a)))
  #print(saat_data)
  test()


def track_stats(data):
  global kilitlenme_merkez_x
  global kilitlenme_merkez_y
  global kilitlenme_bilgisi_genislik
  global kilitlenme_bilgisi_dikey
  global kilitlenme_bilgisi_sonuc
  global kilitlenme_eski_durum
  global kilitlendi_saat
  global kilitlenme_bitti_saat
  kilitlenme_bilgisi = str(data).split(' ')
  kilitlenme_merkez_x = round(float(kilitlenme_bilgisi[1].replace('"','')),2)
  kilitlenme_merkez_y = round(float(kilitlenme_bilgisi[2]),2)
  kilitlenme_bilgisi_genislik = kilitlenme_bilgisi[3]
  kilitlenme_bilgisi_dikey = kilitlenme_bilgisi[4]
  kilitlenme_bilgisi_sonuc = int(kilitlenme_bilgisi[5].replace('"',''))
  if kilitlenme_eski_durum == 0 and kilitlenme_bilgisi_sonuc == 1:
      try:
        kilitlendi_saat = saat_data.split('.')
        print(saat)
      except AttributeError:
        pass
      print('Kilitlendi')
      kilitlenme_eski_durum = 1
  elif kilitlenme_eski_durum == 1 and kilitlenme_bilgisi_sonuc == 0:
      try:
        kilitlenme_bitti_saat = saat_data.split('.')
      except AttributeError:
        pass
      kilitlenme_eski_durum = 0
      print('Gönderiyor',kilitlendi_saat[0],kilitlendi_saat[1],kilitlendi_saat[2],kilitlendi_saat[3])
      #print(kilitlendi_saat[0],kilitlendi_saat[1],kilitlendi_saat[2],kilitlendi_saat[3],kilitlenme_bitti_saat[0],kilitlenme_bitti_saat[1],kilitlenme_bitti_saat[2],kilitlenme_bitti_saat[3],mode_status)
      client.send_kilitlenme_data(kilitlendi_saat[0],kilitlendi_saat[1],kilitlendi_saat[2],kilitlendi_saat[3],kilitlenme_bitti_saat[0],kilitlenme_bitti_saat[1],kilitlenme_bitti_saat[2],kilitlenme_bitti_saat[3],mode_status)
      print('Gönderdi')


 




class Conditions(Enum):
    NotConnected = 0,
    Connected = 1

class Actions(Enum):
    GET_SERVER_TIME = 1,
    POST_SEND_TELEMETRY = 2,
    POST_SEND_LOCK_INFO = 3,
    POST_LOGIN = 4,
    GET_EXIT = 5,

class Client:
    def __init__(self):
        self.team_no = -1
        self.condition = Conditions.NotConnected

        if(os.path.exists("log.txt")):
            os.remove("log.txt")
        self.log_file = open("log.txt","w")

    def log(self, action, status_code, message):
        if(status_code == 200):
            self.log_file.write("[ {0} ] ( Succeded ) : {1}\n".format(Actions(action),message))
        elif(status_code == 204):
            self.log_file.write("[ {0} ] ( Wrong format ) : {1}\n".format(Actions(action),message))
        elif(status_code == 400):
            self.log_file.write("[ {0} ] ( Request is wrong or not valid ) : {1}\n".format(Actions(action),message))
        elif(status_code == 401):
            self.log_file.write("[ {0} ] ( Unauthorized access ) : {1}\n".format(Actions(action),message))
        elif(status_code == 403):
            self.log_file.write("[ {0} ] ( You have no permission to access ) : {1}\n".format(Actions(action),message))
        elif(status_code == 404):
            self.log_file.write("[ {0} ] ( Invalid URL ) : {1}\n".format(Actions(action),message))
        elif(status_code == 500):
            self.log_file.write("[ {0} ] ( Server error ) : {1}\n".format(Actions(action),message))
        else:
            self.log_file.write("[ {0} ] ( Unknown status code ) : {1}\n".format(Actions(action),message))


    def login(self,name,password):
        global cookie
        data = {
            "kadi" : name,
            "sifre" : password
        }
        headers = {'content-type': 'application/json'}
        response = requests.post(url=endpoint+"/api/giris", data=json.dumps(data), headers=headers)
        cookie = str(response.headers).split("Set-Cookie': '")[1].split("'")[0]
        self.log(Actions.POST_LOGIN,response.status_code,response.json())
        f=open('session.txt','w')
        f.write(cookie)
        f.close()
        if(response.status_code == 200):
            print("Login succeded.")
            data = response.json()
            self.team_no = data
            self.condition = Conditions.Connected
            print("Team no : {}".format(self.team_no))
        else:
            print("Login was not successful")
            print("Error : {}".format(response.text))

    def get_server_time(self):
     
        response = requests.get(url=endpoint + "/api/sunucusaati")
        
        self.log(Actions.GET_SERVER_TIME,response.status_code,response.json())
        print(response.content)
        return response.json()

  

    def send_telemetry_data(self,enlem,boylam,irtifa,dikilme,yonelme,yatis,hiz,batarya,iha_otonom,iha_kilitlenme,hedef_x,hedef_y,hedef_genislik,hedef_yukseklik,gps_saat,gps_dakika,gps_saniye,gps_ms):
        headers = {'content-type': 'application/json','cookie':cookie}
        global saniye_eski
        global genislik_eski
        global kilitlenme_eski_durum
        global ms_eski
        telemetry_data = {
            "takim_numarasi": client.team_no,
            "IHA_enlem": enlem,
            "IHA_boylam": boylam,
            "IHA_irtifa": irtifa,
            "IHA_dikilme": dikilme,
            "IHA_yonelme": yonelme,
            "IHA_yatis": yatis,
            "IHA_hiz": hiz,
            "IHA_batarya": int(batarya),
            "IHA_otonom": int(iha_otonom),
            "IHA_kilitlenme": int(iha_kilitlenme),
            "Hedef_merkez_X": int(hedef_x),
            "Hedef_merkez_Y": int(hedef_y),
            "Hedef_genislik": int(hedef_genislik),
            "Hedef_yukseklik": int(hedef_yukseklik),
            "GPSSaati":{
                "saat":int(gps_saat),
                "dakika":int(gps_dakika),
                "saniye":int(gps_saniye),
                "milisaniye":int(gps_ms)
            }
        }
        if int(kilitlenme_eski_durum) != int(iha_kilitlenme) or abs((int(gps_saniye)*1000 + int(gps_ms)) - int(saniye_eski)) >= 800:
          kilitlenme_eski_durum = int(iha_kilitlenme)
          saniye_eski = int(gps_saniye) * 1000 + int(gps_ms)
        #rospy.sleep(1)
          response = requests.post(url=endpoint + "/api/telemetri_gonder", data=json.dumps(telemetry_data), headers=headers)
          #print(response)
          #self.log(Actions.POST_SEND_TELEMETRY, response.status_code, response.json())
          data = response.json()
          print(data,response.status_code,"-"*50)
        
        #server_time = data["sistemSaati"]
        #location_infos = data["konumBilgileri"]

    def send_kilitlenme_data(self,kilitlenme_basla_saat,kilitlenme_basla_dakika,kilitlenme_basla_saniye,kilitlenme_basla_ms,kilitlenme_bitir_saat,kilitlenme_bitir_dakika,kilitlenme_bitir_saniye,kilitlenme_bitir_ms,kilitlenme_otonom):
        headers = {'content-type': 'application/json','cookie':cookie}
        kilitlenme_data = {
            "kilitlenmeBaslangicZamani":{
                "saat":int(kilitlenme_basla_saat),
                "dakika":int(kilitlenme_basla_dakika),
                "saniye":int(kilitlenme_basla_saniye),
                "milisaniye":int(kilitlenme_basla_ms)
            },
            "kilitlenmeBitisZamani":{
                "saat":int(kilitlenme_bitir_saat),
                "dakika":int(kilitlenme_bitir_dakika),
                "saniye":int(kilitlenme_bitir_saniye),
                "milisaniye":int(kilitlenme_bitir_ms)
            },
            "otonom_kilitlenme":int(kilitlenme_otonom)
        }
        print(kilitlenme_data)
        response = requests.post(url=endpoint + "/api/kilitlenme_bilgisi", data=json.dumps(kilitlenme_data), headers=headers)
        print(response.status_code,response.content,"-"*50)

    def close_connection(self):
        headers = {'content-type': 'application/json',"cookie":cookie}
        response = requests.get(url=endpoint + "/api/cikis",headers=headers)
        print(response.content,response.status_code)

    def clean_up(self):
        self.log_file.close()

def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)
  rospy.Subscriber('mavros/global_position/global',NavSatFix,autopilot_gps_status)
  rospy.Subscriber('mavros/global_position/rel_alt',Float64,autopilot_rel_alt_status) 
  rospy.Subscriber('mavros/vfr_hud',VFR_HUD,autopilot_hud_status)
  rospy.Subscriber('mavros/battery',BatteryState,autopilot_battery_status)
  rospy.Subscriber('mavros/state',State,autopilot_state)
  rospy.Subscriber('mavros/time_reference',TimeReference,saat)
  rospy.Subscriber('track',String,track_stats)


def test():
  #print(saat_data)
  if saat_data != 0:
    saat_data_a = saat_data.split('.')
    #print(saat_data_a)
    #print(latitude,longitude ,relative_altitude,pitch,heading_compass,roll,ground_speed ,percentage,mode_status,0,0,0,0,0,saat_data_a[0],saat_data_a[1],saat_data_a[2],saat_data_a[3])
    client.send_telemetry_data(latitude,longitude ,relative_altitude,pitch,heading_compass,roll,ground_speed ,percentage,mode_status, kilitlenme_bilgisi_sonuc,kilitlenme_merkez_x, kilitlenme_merkez_y,kilitlenme_bilgisi_genislik,kilitlenme_bilgisi_dikey,saat_data_a[0],saat_data_a[1],saat_data_a[2],saat_data_a[3])
#client.send_kilitlenme_data(kilitlenme_basla_saat,kilitlenme_basla_dakika,kilitlenme_basla_saniye,kilitlenme_basla_ms,kilitlenme_bitir_saat,kilitlenme_bitir_dakika,kilitlenme_bitir_saniye,kilitlenme_bitir_ms,kilitlenme_otonom)
#client.close_connection()
#client.clean_up()

client = Client()
#client.close_connection()
client.login("gokyuzununmuhendisleri","s56vopc09n")

while not rospy.is_shutdown(): 
  autopilot_listener()  
  test()
  rospy.spin()
