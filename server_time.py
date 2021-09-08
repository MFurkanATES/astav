from flask import jsonify
from datetime import datetime
import flask

app = flask.Flask(__name__)
app.config["DEBUG"] = True
# datetime object containing current date and time


@app.route("/api/sunucusaati")
def summary():
    now = datetime.now()
    dt_string = now.strftime("%H:%M:%S.%f")
    data = {
        "saat" : dt_string.split(':')[0],
        "dakika" : dt_string.split(':')[1],
        "saniye" : dt_string.split(':')[2].split('.')[0],
        "milisaniye" : dt_string.split('.')[1][:3]
    }
    return jsonify(data)
    #return jsonify(saat=dt_string.split(':')[0],dakika=dt_string.split(':')[1],saniye=dt_string.split(':')[2].split('.')[0],milisaniye=dt_string.split('.')[1])

app.run()