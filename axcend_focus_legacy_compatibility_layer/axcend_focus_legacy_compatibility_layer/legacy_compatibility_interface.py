from flask import Flask, request, jsonify
from datetime import datetime
import time
import random
import base64
import socket

app = Flask(__name__)

# Simulating the system state and configuration
system_state = {
    "start_time": time.time(),
    "operator": None,
    "operator_ip": None,
    "reservation_key": None,
    "is_reserved": False,
}


def generate_key(user, ip):
    random_data = random.getrandbits(128)
    key_prefix = "FocusLC"
    key_suffix = "Copyright 2019, Axcend LLC, All Rights Reserved. I understand unauthorized use is illegal."
    encoded_key = base64.b64encode(
        f"{random_data}:{key_prefix}:{user}:{ip}:{key_suffix}".encode()
    ).decode()
    return encoded_key


def generate_json_response(method, result):
    """Generate a JSON response with the method, timestamp, microseconds, and result."""
    return jsonify({
        "method": method,
        "timestamp": datetime.now().strftime("%a %b %d %H:%M:%S %z %Y"),
        "microseconds": str(int(time.time() * 1e6)),
        "result": result,
    })


# Hello world route
@app.route("/")
def home():
    return "Hello, World!"


@app.route("/IsReserved", methods=["GET"])
def is_reserved():
    """Check if the system is reserved."""
    if system_state["is_reserved"]:
        result = {
            "status": "Reserved",
            "operator": system_state["operator"],
            "IP": system_state["operator_ip"],
        }
    else:
        result = {"status": "NOT RESERVED"}

    return generate_json_response("IsReserved", result)


@app.route("/ReserveSystem", methods=["GET"])
def reserve_system():
    """Reserve the system. User is sent as query parameter."""
    user = request.args.get("user")
    ip = request.remote_addr

    if system_state["is_reserved"]:
        info = {"status": "ALREADY RESERVED"}
    else:
        key = generate_key(user, ip)
        system_state.update(
            {
                "operator": user,
                "operator_ip": ip,
                "reservation_key": key,
                "is_reserved": True,
            }
        )
        info = {"status": "RESERVED", "key": key}

    return generate_json_response("ReserveSystem", info)


@app.route("/ReleaseSystem", methods=["GET"])
def release_system():
    """Release the system. Key is sent a query parameter."""
    key = request.args.get("key")

    if not system_state["is_reserved"]:
        status = "NOT RESERVED"
    elif key != system_state["reservation_key"]:
        status = "INVALID KEY"
    else:
        system_state.update(
            {
                "operator": None,
                "operator_ip": None,
                "reservation_key": None,
                "is_reserved": False,
            }
        )
        status = "OK"

    info = {"status": status}

    return generate_json_response("ReleaseSystem", info)


@app.route("/machinename", methods=["GET"])
def machine_name():
    """Return the hostname of the machine."""
    return generate_json_response("machinename", socket.gethostname())


@app.route("/version", methods=["GET"])
def version():
    """Return the version of the system."""
    return generate_json_response("version", "Bridge 3.0.2\n Firmware 3.0.2\n Image 3.0.2\n Config 3.0.2\n")

# read

# write

# cartridge_config

# machinestate

# clear

# deviceconfig

# cartridge_write

# update_system_parameters


if __name__ == "__main__":
    app.run(debug=True)
