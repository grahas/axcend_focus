from flask import Flask, request, jsonify
import random
import base64
import time

app = Flask(__name__)

# Simulating the system state and configuration
system_state = {
    "start_time": time.time(),
    "operator": None,
    "operator_ip": None,
    "reservation_key": None,
    "is_reserved": False
}

def generate_key(user, ip):
    random_data = random.getrandbits(128)
    key_prefix = "FocusLC"
    key_suffix = "Copyright 2019, Axcend LLC, All Rights Reserved. I understand unauthorized use is illegal."
    encoded_key = base64.b64encode(f"{random_data}:{key_prefix}:{user}:{ip}:{key_suffix}".encode()).decode()
    return encoded_key

# Hello world route
@app.route('/')
def home():
    return "Hello, World!"


@app.route('/is_reserved', methods=['GET'])
def is_reserved():
    if system_state['is_reserved']:
        info = {
            "status": "Reserved",
            "operator": system_state['operator'],
            "IP": system_state['operator_ip']
        }
    else:
        info = {"status": "NOT RESERVED"}
    return jsonify(info)

@app.route('/reserve_system', methods=['POST'])
def reserve_system():
    user = request.json.get('user')
    ip = request.remote_addr

    if system_state['is_reserved']:
        info = {"status": "ALREADY RESERVED"}
    else:
        key = generate_key(user, ip)
        system_state.update({
            "operator": user,
            "operator_ip": ip,
            "reservation_key": key,
            "is_reserved": True
        })
        info = {
            "status": "RESERVED",
            "key": key
        }
    return jsonify(info)

@app.route('/release_system', methods=['POST'])
def release_system():
    key = request.json.get('key')

    if not system_state['is_reserved']:
        info = "NOT RESERVED"
    elif key != system_state['reservation_key']:
        info = "INVALID KEY"
    else:
        system_state.update({
            "operator": None,
            "operator_ip": None,
            "reservation_key": None,
            "is_reserved": False
        })
        info = "OK"
    return jsonify(info)

# write




# read




# machinestate




# machinename




# ReserveSystem




# ReleaseSystem




# IsReserved




# version




# clear




# cartridge_config




# ?deviceconfig




# cartridge_write




# update_system_parameters






if __name__ == '__main__':
    app.run(debug=True)
