import requests
import json
cookie = ".AspNetCore.Session=CfDJ8HjCsKwmighDpLgCmEzo9FBY65iH4LSXoh%2B4uEs3hlP3ZIvbi%2BXAI5ypU7OOJ%2FyZiM9foj1%2BGl%2B6oUIkxQPCI93G4G%2F%2F9ubLPsCr47R%2BpkQIhOKs9uwe5ZGYb1gecNQrJ%2B68gokXoS9m3diyZD7CdyDAlbQw4ulg0Y0mEdWQrIax; path=/; samesite=lax; httponly"
def close_connection():
        headers = {'content-type': 'application/json',"cookie":cookie}
        response = requests.get(url="http://192.168.20.10:64559" + "/api/cikis",headers=headers)
        print(response.content,response.status_code)

close_connection()
