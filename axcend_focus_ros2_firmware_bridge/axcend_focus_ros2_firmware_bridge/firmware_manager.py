"""High level wrapper over the firmware to make it easier to use."""

# import debugpy
# debugpy.listen(('0.0.0.0', 5678))
# print("Waiting for debugger attach")
# debugpy.wait_for_client()

import os
import time
import serial
import threading
import queue
import signal
import rclpy
from std_msgs.msg import String
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.action import ActionServer

from axcend_focus_ros2_firmware_bridge.packet_definitions import (
    PacketTranscoder,
    DataAcquisitionState,
)

from axcend_focus_custom_interfaces.srv import CartridgeMemoryWrite
from axcend_focus_custom_interfaces.action import ValveRotate
from axcend_focus_custom_interfaces.msg import CartridgeOvenStatus

# Define some constants

GET_CARTRIDGE_CONFIGURATION_COMMAND_CODE = 0x01

# Globals
firmware_node = None

# Color guide for the comments
# Cartridge_related_functions {#5fb8cf,0}
# Valve_related_functions {#6f4162,0}
# Pump_related_functions {#e172cf,0}
# Oven_related_functions {#09513c, 0}


def firmware_serial_port():
    """Turn on the firmware, Open the serial port."""
    # Variables
    firmware_serial_port_path = "/dev/ttyRPMSG0"

    # Turn on the firmware
    # print("Restarting the firmware")
    # firmware_helper_script_path = os.environ.get("firmware")
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
    """Create a ROS2 node that will translate between the firmware and ROS2."""

    def __init__(self, serial_port=None):
        # Initialize the node
        super().__init__("firmware_node")

        # Create queues for transmitting and receiving packets
        self.transmit_queue = queue.Queue()

        # Load the packet encoding / decoding library
        self.packet_transcoder = PacketTranscoder()

        # Create a default acknowledgement packet
        self.acknowledgement_packet = (
            self.packet_transcoder.create_acknowledgement_packet()
        )

        # Open the serial port, use the dummy testing one if provided
        self.firmware_serial_port = serial_port or firmware_serial_port()

        # Establish RPmsg port
        heart_beat_packet = self.packet_transcoder.create_heartbeat_packet()
        self.firmware_serial_port.write(heart_beat_packet)

        # Used for coordinating the threads lifecycle
        self.is_alive = threading.Event()
        self.is_alive.set()

        # Create a thread to receive data from the firmware
        self.receive_thread_handler = threading.Thread(
            target=self.receive_thread, daemon=True
        )
        self.receive_thread_handler.start()

        # Create a thread to transmit data to the firmware
        self.transmit_thread_handler = threading.Thread(
            target=self.transmit_thread, daemon=True
        )
        self.transmit_thread_handler.start()

        # Create a publisher for raw serial data received from the firmware
        self.publisher = self.create_publisher(String, "firmware_UART_read", 10)

        # Create a subscriber for raw serial data to be sent to the firmware
        self.subscription = self.create_subscription(
            String, "firmware_UART_write", self.listener_callback, 10
        )

        # Create a service to handle the write cartridge memory command {#5fb8cf,5}
        self.cartridge_memory_write_service = self.create_service(
            CartridgeMemoryWrite,
            "cartridge_memory_write",
            self.callback_cartridge_memory_write,
        )

        # Create a service to handle moving the valves {#6f4162,6}
        self.rotate_valves_action = ActionServer(
            self,
            ValveRotate,
            "rotate_valves",
            self.callback_rotate_valves,
        )

        # Create a publisher for oven data {#09513c,3}
        self.temperature_publisher = self.create_publisher(
            CartridgeOvenStatus, "cartridge_oven_state", 10
        )

        # Create a service call to handle manual pump positioning
        # self.manual_pump_positioning_service = self.create_service(
        #     PumpCommand,
        #     "manual_pump_positioning",
        #     self.callback_manual_pump_positioning,
        # )

        # Heart beat related initialization
        self.last_heartbeat_time = time.time()
        self.heartbeat_timeout = 5.0  # Timeout after 5 seconds
        self.check_heartbeat_thread = threading.Thread(
            target=self.check_heartbeat, daemon=True
        )
        self.check_heartbeat_thread.start()

        # Create a dictionary of functions for handling packets
        self.packet_handlers = {
            ("cartridge_config", "DC"): self.handle_cartridge_config_packet,
            ("proto1", "DT"): self.handle_cartridge_oven_status_packet,
            ("proto1", "GH"): self.handle_heart_beat_packet,
            ("proto1", "DA"): self.handle_command_acknowledgement_packet,
            ("proto1", "DN"): self.handle_command_phase_packet,
            ("proto1", "DV"): self.handle_pressure_packet,
        }

        # Send the firmware the system parameters packet
        system_parameters_packet = (
            self.packet_transcoder.create_system_parameters_packet()
        )
        self.transmit_queue.put(system_parameters_packet)

    def check_heartbeat(self):
        """Check if the firmware is still alive."""
        while True:
            time.sleep(1)
            if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
                print("Firmware is not responding!")
                break

    def handle_heart_beat_packet(self, packet):
        """Handle the heart beat packet."""
        self.last_heartbeat_time = time.time()

    def handle_command_phase_packet(self, packet):
        """Receive the change in phase state packet."""
        pass

    def handle_command_acknowledgement_packet(self, packet):
        """Handle the command acknowledgement packet."""
        pass

    # Cartridge related code section {#5fb8cf,26}
    def handle_cartridge_config_packet(self, packet):
        """Deserializing the cartridge config packet and publishing it to the ROS2 topic."""
        pass

    def handle_pressure_packet(self, packet):
        """Handle the pressure packet."""
        pass

    def set_phase(self, phase):
        """Set the phase of the firmware."""
        pass

    def set_operational_mode(self, mode):
        """Set the operational mode of the firmware."""
        pass

    def callback_cartridge_memory_write(self, request, response):
        """Handle the write cartridge memory request."""
        print("Received a write cartridge memory request")

        # Create a cartridge memory packet
        cartridge_memory_string = self.packet_transcoder.create_cartridge_memory_write_packet(
            request
        )

        # Send the packet
        self.transmit_queue.put(cartridge_memory_string)

        # Wait for one second for the acknowledgement
        time.sleep(1)

        response.success = True
        return response

    # Valve related code section {#6f4162,22}
    def callback_rotate_valves(self, goal_handle):
        """Handle the rotate valves request."""
        solvent_valve_position = goal_handle.request.valve_position[0]
        injection_valve_position = goal_handle.request.valve_position[1]

        packet_string = self.packet_transcoder.create_valve_rotate_packet(
            solvent_valve_position, injection_valve_position
        )

        # Send the packet to the firmware
        self.transmit_queue.put(packet_string)

        # Make the results
        result = ValveRotate.Result()
        result.success = True

        # Return a response
        goal_handle.succeed()

        return result

    # Oven related code {#09513c, 30}
    def handle_cartridge_oven_status_packet(self, packet):
        """Deserialize the cartridge temperature packet and publishing it to the ROS2 topic."""
        # Extract the values from the packet
        [sequence_number, oven_state, temperature_as_float, power_output] = (
            self.packet_transcoder.parse_values_oven_status_packet(packet)
        )

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
        temperature_message.current_temperature = temperature_as_float

        # Publish the message
        self.temperature_publisher.publish(temperature_message)

    def transmit_thread(self):
        """Thread is responsible for sending data to the firmware."""
        while rclpy.ok() and self.is_alive.is_set():
            try:
                packet = self.transmit_queue.get(timeout=1)
                if packet:
                    self.firmware_serial_port.write(packet)

            except queue.Empty:
                # No items in the queue this time, just continue with the loop
                continue

    def receive_thread(self):
        """Receive data from the firmware and publish it to the ROS2 topic."""
        while rclpy.ok() and self.is_alive.is_set():
            try:
                received_data = self.firmware_serial_port.readline().decode().strip()
                if received_data:
                    # Publish the raw data
                    msg = String()
                    msg.data = received_data
                    self.publisher.publish(msg)

                    # Decode and dispatch the packet to the appropriate handler
                    [prefix, packet_type, packet] = (
                        self.packet_transcoder.decode_packet(received_data)
                    )
                    self.packet_handlers[(prefix, packet_type)](packet)

            except serial.SerialTimeoutException:
                # No data received this time, just continue with the loop
                continue
            
            except AttributeError:
                # No data received this time, just continue with the loop
                continue

            except Exception as e:
                print(f"Error: {e}")


    def listener_callback(self, msg):
        """Enqueue data to be written to the firmware."""
        self.transmit_queue.put(msg.data.encode())

    def close(self):
        """Clean up the firmware node."""
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
        # print("Turning off the firmware")
        # firmware_helper_script_path = os.environ.get("firmware")
        # os.system(f"{firmware_helper_script_path} stop")

        # Teardown: destroy the node
        print("Destroying the node")
        self.destroy_node()

    def set_data_acquisition_state(self, state: DataAcquisitionState):
        """Set the data acquisition state of the firmware."""
        packet_string = self.packet_transcoder.create_data_acquisition_state_packet(
            state
        )
        self.transmit_queue.put(packet_string)


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
