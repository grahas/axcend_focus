import base64
import random
import socket
import threading
import time
from datetime import datetime
from functools import wraps
from queue import Empty, Queue

import rclpy
from flask import Flask, g, jsonify, request
from rclpy.node import Node
from std_msgs.msg import String

app = Flask(__name__)

# Simulating the system state and configuration
system_state = {
    "start_time": time.time(),
    "operator": None,
    "operator_ip": None,
    "reservation_key": None,
    "is_reserved": False,
    "firmware_UART_read_queue": Queue(),
    "firmware_UART_write_queue": Queue(),
}


class LegacyCompatibilityInterface(Node):
    """Node provides access to the firmware_bridge RX and TX topics."""

    def __init__(self, write_queue: Queue, read_queue: Queue):
        super().__init__("legacy_compatibility_interface")

        self.write_queue = write_queue
        self.read_queue = read_queue
        self.shutdown_event = threading.Event()

        # Create a publisher for the firmware_UART_write topic
        self.firmware_UART_write_publisher = self.create_publisher(
            String, "firmware_UART_write", 10
        )

        # Create a subscriber for the firmware_UART_read topic
        self.firmware_UART_read_subscription = self.create_subscription(
            String, "firmware_UART_read", self.firmware_UART_read_callback, 10
        )

    def firmware_UART_read_callback(self, msg):
        """Callback function for the firmware_bridge TX topic."""
        # Remove the proto1 prefix
        msg.data = msg.data.split("proto1 ")[1]
        self.read_queue.put(msg.data)

    def publish_to_firmware_UART_write_thread(self):
        """Publish messages from the queue to the firmware_UART_write topic."""
        msg = String()
        while not self.shutdown_event.is_set():
            try:
                msg = self.write_queue.get(timeout=0.5)
                self.firmware_UART_write_publisher.publish(msg)
            except Empty:
                continue


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
    return jsonify(
        {
            "method": method,
            "timestamp": datetime.now().strftime("%a %b %d %H:%M:%S %z %Y"),
            "microseconds": str(int(time.time() * 1e6)),
            "result": result,
        }
    )


def check_key(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        key = request.args.get("key")
        if not key:
            key = None

        if key != system_state["reservation_key"]:
            return generate_json_response("write", "INVALID KEY")

        return func(*args, **kwargs)

    return wrapper


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
    return generate_json_response(
        "version", "Bridge 3.0.2\n Firmware 3.0.2\n Image 3.0.2\n Config 3.0.2\n"
    )


@app.route("/read", methods=["GET"])
@check_key
def read():
    """Return all the messages received from the firmware_UART_read topic."""
    # Concatenate and remove all the messages in the read queue
    results = ""
    while not system_state["firmware_UART_read_queue"].empty():
        results += system_state["firmware_UART_read_queue"].get()

    # Prepend the proto1 prefix
    if results != "":
        results = "proto1 " + results

    return generate_json_response("read", results)


@app.route("/write", methods=["GET"])
@check_key
def write():
    """Write the message to the firmware_UART_write topic."""
    message = request.args.get("data")
    msg = String()
    msg.data = message

    # Split the message into 32 character chunks and write them to the topic
    for i in range(0, len(message), 32):
        msg.data = message[i : i + 32]
        msg.data = "proto1 " + msg.data
        system_state["firmware_UART_write_queue"].put(msg)

    return generate_json_response("write", "OK")


# cartridge_config

# machinestate

# clear

# deviceconfig

# cartridge_write

# update_system_parameters


if __name__ == "__main__":
    rclpy.init()
    legacy_compatibility_interface = LegacyCompatibilityInterface(
        system_state["firmware_UART_write_queue"],
        system_state["firmware_UART_read_queue"],
    )

    stop_event = threading.Event()

    flask_thread = threading.Thread(target=app.run, kwargs={"use_reloader": False})
    flask_thread.start()

    # Start the publish thread
    publish_thread = threading.Thread(
        target=legacy_compatibility_interface.publish_to_firmware_UART_write_thread
    )
    publish_thread.start()

    rclpy.spin(legacy_compatibility_interface)
    rclpy.shutdown()
