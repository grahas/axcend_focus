"""
High level wrapper over the firmware to make it easier to use
"""
# import debugpy
# debugpy.listen(('0.0.0.0', 5678))
# print("Waiting for debugger attach")
# debugpy.wait_for_client() 

import os
import time
import serial
import threading
import queue
from collections import defaultdict
from enum import Enum
import json
from threading import Thread
import signal

import rclpy
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.action import ActionServer

from packet_definitions import heart_beat_packet

from axcend_focus_custom_interfaces.srv import CartridgeMemoryWrite
from axcend_focus_custom_interfaces.action import ValveRotate
from axcend_focus_custom_interfaces.msg import CartridgeOvenStatus

# Define some constants
PROTO_PREFIX = b"proto1 "
GET_CARTRIDGE_CONFIGURATION_COMMAND_CODE = 0x01

# Globals
firmware_node = None

import ctypes

PACKET_MAX_LENGTH = 16


class Raw(ctypes.Structure):
    _fields_ = [("raw", ctypes.c_uint8 * PACKET_MAX_LENGTH)]


class Field(ctypes.Structure):
    _fields_ = [
        ("packet_ID", ctypes.c_uint8),
        ("type_ID", ctypes.c_uint8),
        ("unitId", ctypes.c_uint8),
        ("data", ctypes.c_uint8 * 12),
        ("end", ctypes.c_uint8),
    ]


class Packet(ctypes.Union):
    _fields_ = [("raw", Raw), ("field", Field)]


class SystemParametersFields(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),  # Offset 0
        ("hardwareVersion", ctypes.c_uint8),  # Offset 1
        ("valvePumpCtlType", ctypes.c_uint8),  # Offset 2
        ("backPlateBoardType", ctypes.c_uint8),  # Offset 3
        ("boardSerialNumber", ctypes.c_uint32),  # Offset 4
        ("productionDate", ctypes.c_uint32),  # Offset 8
        ("productionLocation", ctypes.c_uint16),  # Offset 12
        ("maxPressureRating", ctypes.c_uint16),  # Offset 14
        ("pressureSensorType", ctypes.c_uint8),  # Offset 16
        ("valveSolType", ctypes.c_uint8),  # Offset 17
        ("valveInjType", ctypes.c_uint8),  # Offset 18
        ("valvePrgType", ctypes.c_uint8),  # Offset 19
        ("pumpSyringeAEmptyValue", ctypes.c_uint16),  # Offset 20
        ("pumpSyringeAFullValue", ctypes.c_uint16),  # Offset 22
        ("pumpSyringeBEmptyValue", ctypes.c_uint16),  # Offset 24
        ("pumpSyringeBFullValue", ctypes.c_uint16),  # Offset 26
        ("pumpLifetimeUsage", ctypes.c_uint32),  # Offset 28
        ("injectionVolume", ctypes.c_uint16),  # Offset 32
        ("pumpSyringesType", ctypes.c_uint8),  # Offset 34
        ("pumpSyringeAVolume", ctypes.c_uint8),  # Offset 35
        ("pumpSyringeBVolume", ctypes.c_uint8),  # Offset 36
    ]


class SystemParameters_t(ctypes.Union):
    _pack_ = 1
    _fields_ = [("raw", ctypes.c_uint8 * 37), ("fields", SystemParametersFields)]


class CartridgeData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("revision", ctypes.c_uint8),
        ("serial_number", ctypes.c_char * 12),
        ("max_pressure_rating", ctypes.c_uint8),
        ("has_oven", ctypes.c_uint8),
        ("is_third_party", ctypes.c_uint8),
        ("last_write_date", ctypes.c_char * 6),
        ("total_run_time", ctypes.c_uint32),
        ("cartridge_flags", ctypes.c_uint8),
        ("detector_count", ctypes.c_uint8),
        ("detector_1_type", ctypes.c_uint8),
        ("detector_1_wavelength", ctypes.c_uint16),
        ("detector_1_path_length", ctypes.c_uint16),
        ("detector_2_type", ctypes.c_uint8),
        ("detector_2_wavelength", ctypes.c_uint16),
        ("detector_2_path_length", ctypes.c_uint16),
        ("column_count", ctypes.c_uint8),
        ("column_1_length", ctypes.c_uint8),
        ("column_1_diameter", ctypes.c_uint16),
        ("column_1_particle_size", ctypes.c_uint8),
        ("column_1_description", ctypes.c_char * 12),
        ("column_2_length", ctypes.c_uint8),
        ("column_2_diameter", ctypes.c_uint16),
        ("column_2_particle_size", ctypes.c_uint8),
        ("column_2_description", ctypes.c_char * 12),
        ("detector_1_adc_gain", ctypes.c_uint8),
        ("detector_1_adc_delay", ctypes.c_uint8),
        ("detector_1_led_pot", ctypes.c_uint8),
        ("detector_2_adc_gain", ctypes.c_uint8),
        ("detector_2_adc_delay", ctypes.c_uint8),
        ("detector_2_led_pot", ctypes.c_uint8),
        ("bluetooth_disabled", ctypes.c_uint8),
        ("report_rate", ctypes.c_uint8),
        ("word_rate", ctypes.c_uint8),
        ("unused", ctypes.c_uint8),
        ("samples_per_point", ctypes.c_uint8),
        ("bits_to_remove", ctypes.c_uint8),
    ]


class CartridgeMemory_t(ctypes.Union):
    _pack_ = 1
    _fields_ = [("raw", ctypes.c_uint8 * 82), ("data", CartridgeData)]


class DataAcquisitionState(Enum):
    """
    Enum is responsible for defining the data acquisition state
    """

    DISABLED = 0
    CARTRIDGE_ONLY = 1
    VALVE_PUMPS_ONLY = 2
    CARTRIDGE_AND_VALVE_PUMPS = 3


def load_packet_transcoder(lib_path: str) -> ctypes.CDLL:
    """
    Load the packet encoding / decoding library and define argument types
    and return types for various functions within the library.
    """
    if not os.path.isfile(lib_path):
        raise FileNotFoundError(f"The library '{lib_path}' does not exist.")

    print("Loading the packet encoding / decoding library")
    packet_transcoder = ctypes.CDLL(lib_path)

    # Define the argument types and return type for each function
    packet_transcoder.packetEncode_16.argtypes = [
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.POINTER(Packet),
        ctypes.c_uint16,
    ]
    packet_transcoder.packetEncode_16.restype = ctypes.c_int

    packet_transcoder.packetEncode_16_16_b.argtypes = [
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.POINTER(Packet),
        ctypes.c_uint16,
        ctypes.c_uint16,
    ]
    packet_transcoder.packetEncode_16_16_b.restype = ctypes.c_int

    packet_transcoder.packetEncode_text.argtypes = [
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.POINTER(Packet),
        ctypes.c_char_p,
    ]
    packet_transcoder.packetEncode_text.restype = ctypes.c_int

    packet_transcoder.packetEncode_8_32.argtypes = [
        ctypes.POINTER(Packet),
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint32,
    ]
    packet_transcoder.packetEncode_8_32.restype = ctypes.c_int

    packet_transcoder.packetDecode_oven_data.argtypes = [
        ctypes.POINTER(Packet), 
        ctypes.POINTER(ctypes.c_uint8), 
        ctypes.POINTER(ctypes.c_uint8), 
        ctypes.POINTER(ctypes.c_uint32), 
        ctypes.POINTER(ctypes.c_uint8), 
        ctypes.POINTER(ctypes.c_uint32), 
        ctypes.POINTER(ctypes.c_uint8)
    ]
    packet_transcoder.packetDecode_oven_data.restype = ctypes.c_int

    return packet_transcoder


def system_parameters_packet() -> str:
    """
    Make the system parameters packet
    """
    # Variables
    system_parameters_file_path = os.environ.get("SYS_PARAMS_FILE")

    # Open the file and load the JSON object
    with open(system_parameters_file_path, "r") as f:
        data = json.load(f)

    # Create an instance of the SystemParameters_t union
    params = SystemParameters_t()

    # Set the fields of the union according to the data in the JSON object
    params.fields.version = int(data["version"])
    params.fields.hardwareVersion = int(data["hardwareVersion"])
    params.fields.valvePumpCtlType = int(data["valvePumpCtlType"])
    params.fields.backPlateBoardType = int(data["backPlateBoardType"])
    params.fields.boardSerialNumber = int(data["boardSerialNumber"])
    params.fields.productionDate = int(data["productionDate"])
    params.fields.productionLocation = int(data["productionLocation"])
    params.fields.maxPressureRating = int(data["maxPressureRating"])
    params.fields.pressureSensorType = int(data["pressureSensorType"])
    params.fields.valveSolType = int(data["valveSolType"])
    params.fields.valveInjType = int(data["valveInjType"])
    params.fields.valvePrgType = int(data["valvePrgType"])
    params.fields.pumpSyringeAEmptyValue = int(data["pumpSyringeAEmptyValue"])
    params.fields.pumpSyringeAFullValue = int(data["pumpSyringeAFullValue"])
    params.fields.pumpSyringeBEmptyValue = int(data["pumpSyringeBEmptyValue"])
    params.fields.pumpSyringeBFullValue = int(data["pumpSyringeBFullValue"])
    params.fields.pumpLifetimeUsage = int(data["pumpLifetimeUsage"])
    params.fields.injectionVolume = int(data["injectionVolume"])
    params.fields.pumpSyringesType = int(data["pumpSyringesType"])
    params.fields.pumpSyringeAVolume = int(data["pumpSyringeAVolume"])
    params.fields.pumpSyringeBVolume = int(data["pumpSyringeBVolume"])

    # Serialize the struct into a hex encoded string
    packet_string = b"SysParams "
    packet_string += bytes(params.raw).hex().upper().encode()

    return packet_string


def acknowledgement_packet(packet_transcoder: ctypes.CDLL) -> str:
    """
    Create a default acknowledgement packet
    """
    ACK_VALUE = b"OK"

    ack_packet = Packet()

    # Make an OK packet
    packet_ID = ctypes.c_uint8(ord("D"))
    type_ID = ctypes.c_uint8(ord("A"))
    buf = ctypes.create_string_buffer(ACK_VALUE, len(ACK_VALUE))
    packet_transcoder.packetEncode_text(
        packet_ID, type_ID, ctypes.byref(ack_packet), buf
    )

    # Serialize the packet into a hex encoded string
    packet_string = PROTO_PREFIX + bytes(ack_packet.raw).hex().upper().encode()
    return packet_string


def firmware_serial_port():
    """
    Turn on the firmware
    Open the serial port
    """
    # Variables
    firmware_serial_port_path = "/dev/ttyRPMSG0"

    # Turn on the firmware
    print("Restarting the firmware")
    firmware_helper_script_path = os.environ.get("firmware")
    # os.system(f"{firmware_helper_script_path} restart")

    # Delay to allow the firmware to restart
    print("Waiting for the firmware to restart")
    time.sleep(2)
    print("Done waiting for the firmware to restart")

    # Set up: open the serial port
    port = serial.Serial(firmware_serial_port_path, timeout=1)

    # Return the prot
    return port


class FirmwareNode(Node):
    """
    Create a ROS2 node that is responsible for translating firmware packets to ROS2 topics
    """

    def __init__(self):
        # Initialize the node
        super().__init__("firmware_node")

        # Create queues for transmitting and receiving packets
        self.transmit_queue = queue.Queue()

        # Load the packet encoding / decoding library
        lib_path = "/axcend/tests/packets.so"
        self.packet_transcoder = load_packet_transcoder(lib_path)

        # Create a default acknowledgement packet
        self.acknowledgement_packet = acknowledgement_packet(self.packet_transcoder)

        # Open the serial port
        self.firmware_serial_port = firmware_serial_port()

        # Establish RPmsg port
        self.firmware_serial_port.write(heart_beat_packet)

        # Used for coordinating the threads lifecycle
        self.is_alive = threading.Event()
        self.is_alive.set()

        # Create a thread to receive data from the firmware
        self.receive_thread_handler = threading.Thread(target=self.receive_thread, daemon=True)
        self.receive_thread_handler.start()

        # Create a thread to transmit data to the firmware
        self.transmit_thread_handler = threading.Thread(target=self.transmit_thread, daemon=True)
        self.transmit_thread_handler.start()

        # Create a dictionary of functions for handling packets
        self.packet_handlers = {
            ("cartridge_config", "DC"): self.handle_cartridge_config_packet,
            ("proto1", "DT"): self.handle_cartridge_oven_status_packet,
            ("proto1", "GH"): self.handle_heart_beat_packet,
            ("proto1", "DA"): self.handle_command_acknowledgement_packet,
            ("proto1", "DN"): self.handle_command_phase_packet,
            ("proto1", "DV"): self.handle_pressure_packet,
        }

        # Create publishers for the different types of data the firmware produces
        self.temperature_publisher = self.create_publisher(
            CartridgeOvenStatus, "cartridge_oven_state", 10
        )

        # Send the firmware the system parameters packet
        self.transmit_queue.put(system_parameters_packet())

        # Create action servers for the different types of commands the firmware can receive
        # self.data_acquisition_action_server = ActionServer(
        #     self,
        #     DataAcquisition,
        #     "data_acquisition",
        #     self.handle_data_acquisition_command)

        # Create a service to handle the write cartridge memory command
        self.cartridge_memory_write_service = self.create_service(
            CartridgeMemoryWrite,
            "cartridge_memory_write",
            self.callback_cartridge_memory_write,
        )

        # Create a service to handle moving the valves
        self.rotate_valves_action = ActionServer(
            self,
            ValveRotate,
            "rotate_valves",
            self.callback_rotate_valves,
        )

        # Create a service call to handle manual pump positioning
        # self.manual_pump_positioning_service = self.create_service(
        #     PumpCommand,
        #     "manual_pump_positioning",
        #     self.callback_manual_pump_positioning,
        # )

    def close(self):
        """
        Function is responsible for cleaning up the firmware node
        """
        # Teardown: join the receive transmit threads
        print("Joining the threads")
        self.is_alive.clear()
        self.receive_thread_handler.join()
        self.transmit_thread_handler.join()
        print("Done joining the threads")

        # Teardown: close the serial port
        print("Closing the firmware serial port")
        self.firmware_serial_port.close()

        # Teardown: turn off the firmware
        print("Turning off the firmware")
        firmware_helper_script_path = os.environ.get("firmware")
        os.system(f"{firmware_helper_script_path} stop")

        # Teardown: destroy the node
        print("Destroying the node")
        self.destroy_node()

    def parse_packet(self, line_of_data):
        """
        Function is responsible for parsing a packet received from the firmware
        """
        prefix, data = line_of_data.split(" ", 1)

        # Convert the hex encoded data to byte array
        data = bytes.fromhex(data)

        # Get the packet type
        packet_type = chr(data[0]) + chr(data[1])

        if prefix == "proto1":
            # Make sure the packet is the correct size
            assert len(data) == 16, "Invalid packet size for proto1"  # 16 bytes

            # Move the data into a packet object
            packet = Packet()

            # Copy the bytes into the packet
            ctypes.memmove(ctypes.byref(packet.raw), data, len(data))

            # Dispatch the packet to the correct handler
            self.packet_handlers[(prefix, packet_type)](packet)

        elif prefix == "cartridge_config":
            assert (
                len(data) == 84
            ), "Invalid packet size for cartridge_config"  # 84 bytes
            packet_type = "cartridge_config"

            try:
                self.packet_handlers[(prefix, packet_type)](data)
            except KeyError:
                print(f"Unhandled packet type: {packet_type}")

    def send_proto1_packet(self, packet: Packet):
        """
        Function is responsible for loading a packet into the transmit queue
        to be sent to the firmware
        """
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        self.transmit_queue.put(packet_string)

    def receive_thread(self):
        """
        Thread is responsible for receiving data from the firmware
        """
        while self.is_alive.is_set():
            try:
                line_of_data = self.firmware_serial_port.readline().decode().strip()
                if line_of_data:
                    self.parse_packet(line_of_data)
            except serial.SerialTimeoutException:
                # No data received this time, just continue with the loop
                continue

    def transmit_thread(self):
        """
        Thread is responsible for sending data to the firmware
        """
        while self.is_alive.is_set():
            try:
                packet = self.transmit_queue.get(timeout=1)
                if packet:
                    self.firmware_serial_port.write(packet)
            except queue.Empty:
                # No items in the queue this time, just continue with the loop
                continue

    def callback_manual_pump_positioning(self, request, response):
        """
        Manually position the pumps
        """
        # Create a packet
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("D"))
        type_ID = ctypes.c_uint8(ord("P"))
        buf = ctypes.create_string_buffer(request.data, len(request.data))
        self.packet_transcoder.packetEncode_text(
            packet_ID, type_ID, ctypes.byref(packet), buf
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()

        # Send the packet to the firmware
        self.transmit_queue.put(packet_string)

        # Return a response
        response.success = True
        return response

    def callback_cartridge_memory_write(self, request, response):
        """
        Function is responsible for handling the write cartridge memory request
        """
        print("Received a write cartridge memory request")
        print(request)

        # Create a cartridge memory packet
        cartridge_memory = CartridgeMemory_t()

        cartridge_memory.data.version = request.version
        cartridge_memory.data.revision = request.revision
        cartridge_memory.data.serial_number = request.serial_number.encode()
        cartridge_memory.data.has_oven = request.has_oven
        cartridge_memory.data.max_pressure_rating = request.max_pressure_rating

        # Serialize the packet into a hex encoded string
        cartridge_params_packet_string = b"cartridge_config "
        cartridge_params_packet_string += (
            bytes(cartridge_memory.raw).hex().upper().encode()
        )
        print(cartridge_params_packet_string.decode())

        # Send the packet
        self.transmit_queue.put(cartridge_params_packet_string)

        # Wait for one second for the acknowledgement
        time.sleep(1)
        
        response.success = True
        return response

    def callback_rotate_valves(self, goal_handle):
        """
        Function is responsible for handling the rotate valves request
        """
        # Create a packet
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("C"))
        type_ID = ctypes.c_uint8(ord("V"))
        solvent_valve_position = goal_handle.request.valve_position[0]
        injection_valve_position = goal_handle.request.valve_position[1]

        valve_position = (injection_valve_position << 2) | solvent_valve_position

        # Call packetEncode_16_16_b
        self.packet_transcoder.packetEncode_16_16_b(
            packet_ID,
            type_ID,
            ctypes.byref(packet),
            ctypes.c_uint16(0),
            ctypes.c_uint16(valve_position),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX
        packet_string += bytes(packet.raw).hex().upper().encode()
        print(packet_string.decode())

        # Print the valve target locations
        print(
            f"Solvent valve position: {str(solvent_valve_position)} \t Injection valve position: {str(injection_valve_position)}"
        )

        # Send the packet to the firmware
        self.transmit_queue.put(packet_string)

        # Make the results
        result = ValveRotate.Result()
        result.success = True

        # Return a response
        goal_handle.succeed()

        return result


    def set_operational_mode(self, mode):
        """
        Sets the operational mode of the firmware
        """
        pass

    def set_phase(self, phase):
        """
        Sets the phase of the firmware
        """
        pass

    def set_data_acquisition_state(self, state: DataAcquisitionState):
        """
        Sets the data acquisition state of the firmware
        """
        # Create a blank packet
        packet = Packet()

        # Enable temperature sensor output
        self.packet_transcoder.packetEncode_16(
            ctypes.c_uint8(ord("C")),
            ctypes.c_uint8(ord("U")),
            ctypes.byref(packet),
            ctypes.c_uint16(state.value),
        )

        # Send the packet to the firmware
        self.send_proto1_packet(packet)

    def handle_heart_beat_packet(self, packet):
        """
        Function responsible for handling the heart beat packet
        """
        pass

    def handle_command_phase_packet(self, packet):
        """
        Function is responsible for receiving the change in phase state packet
        """
        pass

    def handle_command_acknowledgement_packet(self, packet):
        """
        Function is responsible for handling the command acknowledgement packet
        """
        pass

    def handle_cartridge_config_packet(self, packet):
        """
        Function is responsible for deserializing the cartridge config
        packet and publishing it to the ROS2 topic
        """
        pass

    def handle_cartridge_oven_status_packet(self, packet: Packet):
        """
        Function is responsible for deserializing the cartridge temperature
        packet and publishing it to the ROS2 topic
        """
        # Extract the data from the packet
        sequence_number = ctypes.c_uint32(0)
        oven_state = ctypes.c_uint8(0)
        temperature = ctypes.c_uint16(0)
        power_output = ctypes.c_uint8(0)

        self.packet_transcoder.packetDecode_oven_data(
            ctypes.byref(packet),
            ctypes.byref(sequence_number),
            ctypes.byref(oven_state),
            ctypes.byref(temperature),
            ctypes.byref(power_output),
        )
        temperature_as_float = temperature.value / 100

        # Create a temperature message
        temperature_message = CartridgeOvenStatus()

        # Set the header
        temperature_message.header = Header()
        temperature_message.header.stamp.sec = sequence_number.value // 1000
        temperature_message.header.stamp.nanosec = int(
            (sequence_number.value % 1000) * 1e6
        )
        temperature_message.header.frame_id = "cartridge"

        # Set the oven state
        temperature_message.oven_state = oven_state.value

        # Set the power output
        temperature_message.power_output = power_output.value

        # Set the temperature
        temperature_message.temperature = temperature_as_float

        # Publish the message
        self.temperature_publisher.publish(temperature_message)

    def handle_pressure_packet(self, packet):
        """
        Function is responsible for handling the pressure packet
        """
        pass

def signal_handler(signum, frame):
    firmware_node.close()
    rclpy.shutdown()

if __name__ == "__main__":
    # Start ROS 2 client library
    rclpy.init()

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # Add nodes to executor
    firmware_node = FirmwareNode()
    executor.add_node(firmware_node)

    # Register SIGINT handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Spin in a separate thread
    # executor_thread = Thread(target=executor.spin, daemon=True)
    # executor_thread.start()
    rclpy.spin(firmware_node)

    firmware_node.close()
    rclpy.shutdown()
    # executor_thread.join()

    # ros2 action send_goal /rotate_valves axcend_focus_custom_interfaces/action/ValveRotate "{valve_position: [0, 1]}"
    # pkill python
