import requests
import json
cookie = ".AspNetCore.Session=CfDJ8HjCsKwmighDpLgCmEzo9FC5k8YNLEO5aO8Q6MduEdqqtDnivnNOP%2FKs6mDaD%2BbdJsXrHIi32bwrunyiQiKNhJ2oD1owIaWLAQOZ16uMPtDb3yrfZ4TO8NgWwP%2B39dU9HWcOe7l5h0mqz%2FMCzKqxVZyoSjgZHcjiVOpTCNLrjsUF; path=/; samesite=lax; httponly"
def close_connection():
        headers = {'content-type': 'application/json',"cookie":cookie}
        response = requests.get(url="http://192.168.20.10:64559" + "/api/cikis",headers=headers)
        print(response.content,response.status_code)

#headers = {'content-type': 'application/json','cookie':cookie}
def get_server_time():
        global sunucu_saat
        response = requests.get(url="http://127.0.0.1:5000/" + "/api/sunucusaati")#,headers=headers)
        #self.log(Actions.GET_SERVER_TIME,response.status_code,response.json())
        print(response.content)
        #return response.json()
#close_connection()
get_server_time()