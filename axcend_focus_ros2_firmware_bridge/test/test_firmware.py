import debugpy
debugpy.listen(('0.0.0.0', 5678))
debugpy.wait_for_client() 

import csv
import pytest
import os
import time
import json
import itertools
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from threading import Thread

from axcend_focus.axcend_focus_ros2_firmware_bridge.axcend_focus_ros2_firmware_bridge.firmware_manager import FirmwareNode, DataAcquisitionState
from axcend_focus_custom_interfaces.srv import CartridgeMemoryWrite


class ExampleNode(Node):
    """
    Node is for use of the firmware test script
    """
    def __init__(self):
        super().__init__('test_firmware_node')
        self.subscription = self.create_subscription(
            Temperature,
            'cartridge_temperature',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.msg_data = None
        self.cartridge_temperature = deque(maxlen=10)

        # Create a client to access the write cartridge memory service
        self.cartridge_memory_write_client = self.create_client(
            CartridgeMemoryWrite, 'cartridge_memory_write')


    def listener_callback(self, msg):
        self.cartridge_temperature.append(msg.temperature)


@pytest.fixture(scope="module")
def nodes():
    # Start ROS 2 client library
    rclpy.init()

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # Add nodes to executor
    firmware_node = FirmwareNode()
    test_node = ExampleNode()
    executor.add_node(firmware_node)
    executor.add_node(test_node)

    # Spin in a separate thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    yield firmware_node, test_node

    firmware_node.close()
    rclpy.shutdown()
    executor_thread.join()


@pytest.fixture(scope="module")
def valves():
    """
    Wait for the valves to complete their initialization
    Read from the firmware logs to determine when the valves are done homing
    """
    print("Waiting for valves to home")
    valves_homed = False
    while not valves_homed:
        # Read from the file at /sys/kernel/debug/remoteproc/remoteproc0/trace0
        # Look for the "Done homing the valves" message
        with open("/sys/kernel/debug/remoteproc/remoteproc0/trace0", "r") as f:
            for line in f:
                if "Done homing the valves" in line:
                    valves_homed = True
                    break
        time.sleep(0.5)
    print("Valves homed")
    yield valves_homed


# @pytest.mark.skip(reason="Not interesting right now")
# def test_packet_encoding(packet_transcoder):
#     """
#     Verify that the packet encoding library is working correctly
#     """
#     # Create a packet
#     packet = Packet()

#     # Convert characters to uint8_t
#     packet_ID = ctypes.c_uint8(ord("C"))
#     type_ID = ctypes.c_uint8(ord("Q"))

#     # Call packetEncode_16
#     packet_transcoder.packetEncode_16(
#         packet_ID, type_ID, ctypes.byref(packet), ctypes.c_uint16(0)
#     )

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(bytes(packet.raw).hex())
#     print(packet_string.decode())

#     # Check the encoding
#     assert (
#         packet_string == b"proto1 4351000000000000000000000000000A"
#     ), "Packet not encoded correctly"


# # @pytest.mark.skip(reason="Not interesting right now")
# def test_heart_beat(serial_port):
#     """
#     Verify that the firmware is able to respond to the
#     heartbeat packet.
#     """
#     heart_beat_packet = b"proto1 4748000000000000000000000000000A"
#     serial_port.write(heart_beat_packet)

#     # Read from the serial port for one second
#     read_results = serial_port.read(1024)
#     assert (
#         b"proto1 4748000000000000000000000000000A" in read_results
#     ), "Heartbeat packet not received"


# @pytest.mark.skip(reason="Not interesting right now")
# def test_abort(serial_port, packet_transcoder):
#     """
#     Verify that the firmware is able to handle the abort and
#     move to a safe state
#     """
#     # Create an abort packet
#     packet = Packet()

#     # Convert characters to uint8_t
#     packet_ID = ctypes.c_uint8(ord("C"))
#     type_ID = ctypes.c_uint8(ord("Q"))

#     # Call packetEncode_16
#     packet_transcoder.packetEncode_16(
#         packet_ID, type_ID, ctypes.byref(packet), ctypes.c_uint16(0)
#     )

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # Send the packet
#     serial_port.write(packet_string)

#     # Read from the serial port for one second
#     read_results = serial_port.read(1024)
#     assert (
#         b"proto1 4451000000000000000000000000000A" in read_results
#     ), "Abort packet not acknowledged"


# @pytest.mark.skip(reason="Not interesting right now")
# def test_system_parameters():
#     """
#     Verify that the device is able to receive system parameters
#     """
#     # Variables
#     system_parameters_file_path = os.environ.get("SYS_PARAMS_FILE")

#     # Open the file and load the JSON object
#     with open(system_parameters_file_path, "r") as f:
#         data = json.load(f)

#     # Create an instance of the SystemParameters_t union
#     params = SystemParameters_t()

#     # Set the fields of the union according to the data in the JSON object
#     params.fields.version = int(data["version"])
#     params.fields.hardwareVersion = int(data["hardwareVersion"])
#     params.fields.valvePumpCtlType = int(data["valvePumpCtlType"])
#     params.fields.backPlateBoardType = int(data["backPlateBoardType"])
#     params.fields.boardSerialNumber = int(data["boardSerialNumber"])
#     params.fields.productionDate = int(data["productionDate"])
#     params.fields.productionLocation = int(data["productionLocation"])
#     params.fields.maxPressureRating = int(data["maxPressureRating"])
#     params.fields.pressureSensorType = int(data["pressureSensorType"])
#     params.fields.valveSolType = int(data["valveSolType"])
#     params.fields.valveInjType = int(data["valveInjType"])
#     params.fields.valvePrgType = int(data["valvePrgType"])
#     params.fields.pumpSyringeAEmptyValue = int(data["pumpSyringeAEmptyValue"])
#     params.fields.pumpSyringeAFullValue = int(data["pumpSyringeAFullValue"])
#     params.fields.pumpSyringeBEmptyValue = int(data["pumpSyringeBEmptyValue"])
#     params.fields.pumpSyringeBFullValue = int(data["pumpSyringeBFullValue"])
#     params.fields.pumpLifetimeUsage = int(data["pumpLifetimeUsage"])
#     params.fields.injectionVolume = int(data["injectionVolume"])
#     params.fields.pumpSyringesType = int(data["pumpSyringesType"])
#     params.fields.pumpSyringeAVolume = int(data["pumpSyringeAVolume"])
#     params.fields.pumpSyringeBVolume = int(data["pumpSyringeBVolume"])

#     # Serialize the struct into a hex encoded string
#     packet_string = b"SysParams "
#     packet_string += bytes(params.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # Send the packet
#     firmware_node.transmit_queue.put(packet_string)

    # # Create the expected response
    # # Make an OK packet
    # packet = Packet()
    # packet_ID = ctypes.c_uint8(ord("D"))
    # type_ID = ctypes.c_uint8(ord("N"))
    # buf = ctypes.create_string_buffer(b"OK", 13)
    # packet_transcoder.packetEncode_text(packet_ID, type_ID, ctypes.byref(packet), buf)

    # # Serialize the packet into a hex encoded string
    # packet_string = PROTO_PREFIX
    # packet_string += bytes(packet.raw).hex().upper().encode()
    # print(packet_string.decode())

    # # Read from the serial port for one second
    # read_results = serial_port.read(1024)
    # assert packet_string in read_results, "System parameters not acknowledged"


# @pytest.mark.skip(reason="Not interesting right now")
# def test_valve_homing(serial_port, packet_transcoder):
#     """
#     Verify that the valve is able to home its self
#     """
#     # Create a valve homing packet
#     packet = Packet()

#     # Convert characters to uint8_t
#     packet_ID = ctypes.c_uint8(ord("C"))
#     type_ID = ctypes.c_uint8(ord("V"))

#     # Call packetEncode_16_16_b
#     packet_transcoder.packetEncode_16_16_b(
#         packet_ID, type_ID, ctypes.byref(packet), ctypes.c_uint16(1), ctypes.c_uint16(0)
#     )

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # Send the packet
#     serial_port.write(packet_string)

#     # Create the expected response
#     # Make an OK packet
#     packet_ID = ctypes.c_uint8(ord("D"))
#     type_ID = ctypes.c_uint8(ord("N"))
#     buf = ctypes.create_string_buffer(b"OK", 13)
#     packet_transcoder.packetEncode_text(packet_ID, type_ID, ctypes.byref(packet), buf)

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # The valves can take upto 30 seconds to home
#     # Wait for the valve to home
#     time.sleep(30)

#     # Read from the serial port for one second
#     read_results = serial_port.read(1024)
#     assert packet_string in read_results, "Valve packet not acknowledged"


# @pytest.mark.skip(reason="Not interesting right now")
# def test_valve_move(serial_port, packet_transcoder, valves):
#     """
#     Verify that the valve is able to move to a position
#     """
#     # Valve positions
#     solvent_valve_positions = range(0, 4)
#     injection_valve_positions = range(0, 2)

#     # Move the valves in all possible combinations
#     for solvent_valve_position, injection_valve_position in itertools.product(
#         solvent_valve_positions, injection_valve_positions
#     ):
#         # Create a valve move packet
#         packet = Packet()

#         # Convert characters to uint8_t
#         packet_ID = ctypes.c_uint8(ord("C"))
#         type_ID = ctypes.c_uint8(ord("V"))
#         valve_position = (injection_valve_position << 2) | solvent_valve_position

#         # Call packetEncode_16_16_b
#         packet_transcoder.packetEncode_16_16_b(
#             packet_ID,
#             type_ID,
#             ctypes.byref(packet),
#             ctypes.c_uint16(0),
#             ctypes.c_uint16(valve_position),
#         )

#         # Serialize the packet into a hex encoded string
#         packet_string = PROTO_PREFIX
#         packet_string += bytes(packet.raw).hex().upper().encode()
#         print(packet_string.decode())

#         # Print the valve target locations
#         print(
#             f"Solvent valve position: {str(solvent_valve_position)} \t Injection valve position: {str(injection_valve_position)}"
#         )

#         # Send the packet
#         serial_port.write(packet_string)

#         # Create the expected response
#         # Make an OK packet
#         packet_ID = ctypes.c_uint8(ord("D"))
#         type_ID = ctypes.c_uint8(ord("N"))
#         buf = ctypes.create_string_buffer(b"OK", 13)
#         packet_transcoder.packetEncode_text(
#             packet_ID, type_ID, ctypes.byref(packet), buf
#         )

#         # Serialize the packet into a hex encoded string
#         packet_string = PROTO_PREFIX
#         packet_string += bytes(packet.raw).hex().upper().encode()
#         print(packet_string.decode())

#         # The valve should take less than 2 seconds to move
#         # Wait for the valve to move
#         time.sleep(2)

#         # Read from the serial port for one second
#         read_results = serial_port.read(1024)
#         assert packet_string in read_results, "Valve packet not acknowledged"


# @pytest.mark.skip(reason="Not interesting right now")
# def test_valve_repeat_move(serial_port, packet_transcoder, valves):
#     """
#     Check that the valve is able to repeatedly move to a position
#     """
#     injection_valve_positions = range(0, 2)

#     for _ in range(5):
#         for injection_valve_position in injection_valve_positions:
#             # Only test the injection valve
#             solvent_valve_position = 0

#             # Create a valve move packet
#             packet = Packet()

#             # Convert characters to uint8_t
#             packet_ID = ctypes.c_uint8(ord("C"))
#             type_ID = ctypes.c_uint8(ord("V"))
#             valve_position = (injection_valve_position << 2) | solvent_valve_position

#             # Call packetEncode_16_16_b
#             packet_transcoder.packetEncode_16_16_b(
#                 packet_ID,
#                 type_ID,
#                 ctypes.byref(packet),
#                 ctypes.c_uint16(0),
#                 ctypes.c_uint16(valve_position),
#             )

#             # Serialize the packet into a hex encoded string
#             packet_string = PROTO_PREFIX
#             packet_string += bytes(packet.raw).hex().upper().encode()
#             print(packet_string.decode())

#             # Print the valve target locations
#             print(
#                 f"Solvent valve position: {str(solvent_valve_position)} \t Injection valve position: {str(injection_valve_position)}"
#             )

#             # Send the packet
#             serial_port.write(packet_string)

#             # Create the expected response
#             # Make an OK packet
#             packet_ID = ctypes.c_uint8(ord("D"))
#             type_ID = ctypes.c_uint8(ord("N"))
#             buf = ctypes.create_string_buffer(b"OK", 13)
#             packet_transcoder.packetEncode_text(
#                 packet_ID, type_ID, ctypes.byref(packet), buf
#             )

#             # Serialize the packet into a hex encoded string
#             packet_string = PROTO_PREFIX
#             packet_string += bytes(packet.raw).hex().upper().encode()
#             print(packet_string.decode())

#             # The valve should take less than 1 seconds to move
#             # Wait for the valve to move
#             time.sleep(1)

#             # Read from the serial port for one second
#             read_results = serial_port.read(1024)
#             assert packet_string in read_results, "Valve packet not acknowledged"


# @pytest.mark.skip(reason="Not interesting right now")
def test_cartridge_memory_write_read(nodes):
    """
    Function is responsible for checking that the cartridge memory is written correctly
    and that the cartridge memory can be read back correctly
    """

    firmware_node, test_node = nodes

    request = CartridgeMemoryWrite.Request()
    request.version = 3
    request.revision = 1
    request.serial_number = "ABC!#&def456"
    request.has_oven = 1
    request.max_pressure_rating = 60 # Hundreds of PSI

    future = test_node.cartridge_memory_write_client.call_async(request)
    rclpy.spin_until_future_complete(test_node, future)
    if future.result() is not None:
        print(f'Result from cartridge_memory_write_service: {future.result().success}')
    else:
        print('Exception while calling cartridge_memory_write_service')

    # # Check for an OK packet
    # read_results = serial_port.read(1024)
    # assert (
    #     acknowledgement_packet in read_results
    # ), "Cartridge memory write packet not acknowledged"

    # # Create a cartridge memory read packet
    # packet = Packet()

    # # Create an array of 13 integers
    # arr = (ctypes.c_int * 13)()

    # # Set the first integer to 1
    # arr[0] = GET_CARTRIDGE_CONFIGURATION_COMMAND_CODE
    # packet_transcoder.packetEncode_text(
    #     ctypes.c_uint8(ord("C")),
    #     ctypes.c_uint8(ord("A")),
    #     ctypes.byref(packet),
    #     bytes(arr),
    # )

    # # Serialize the packet into a hex encoded string
    # packet_string = PROTO_PREFIX
    # packet_string += bytes(packet.raw).hex().upper().encode()
    # print(packet_string.decode())

    # # Send the packet
    # serial_port.write(packet_string)

    # # Check for an OK packet
    # read_results = serial_port.read(1024)
    # assert (
    #     acknowledgement_packet in read_results
    # ), "Cartridge memory read packet not acknowledged"
    # print(read_results)

    # cartridge_params_packet_string = b"cartridge_config " + "DC".encode().hex().upper().encode()
    # cartridge_params_packet_string += bytes(cartridge_memory.raw).hex().upper().encode()

    # # Check that the cartridge memory is correct
    # # read_results = serial_port.read(1024)
    # assert (
    #     cartridge_params_packet_string in read_results
    # ), "Cartridge memory read packet not correct"


@pytest.mark.skip(reason="Not interesting right now")
def test_temperature_sensor(nodes, valves):
    """
    Function is responsible for checking that the temperature sensor can be read
    and is within a reasonable range
    """
    firmware_node, test_node = nodes

    # Set the data acquisition state to cartridge only
    firmware_node.set_data_acquisition_state(DataAcquisitionState.CARTRIDGE_ONLY)

    # Read the temperature sensor output for 10 seconds
    # Check that the temperature is within a reasonable range
    test_node.cartridge_temperature.clear()
    start_time = time.time()
    while (time.time() - start_time) < 30:
    # for _ in range(10):
        try:
            temperature = test_node.cartridge_temperature.popleft()
            print(f"Cartridge temperature: {temperature}")
            assert temperature > -20, "Temperature is too low"
            assert temperature < 100, "Temperature is too high"
        except IndexError:
            pass
        finally:
            time.sleep(1)

    # Set the data acquisition state to disabled
    firmware_node.set_data_acquisition_state(DataAcquisitionState.DISABLED)


# @pytest.mark.skip(reason="Not interesting right now")
# def test_oven_fan(serial_port, packet_transcoder, acknowledgement_packet):
#     """
#     Function is responsible for toggling the oven fan on and off
#     """
#     # Create a blank packet
#     packet = Packet()

#     # Create a packet to turn the oven fan on
#     packet_transcoder.packetEncode_8_32(
#         ctypes.byref(packet),
#         ctypes.c_uint8(ord("C")),
#         ctypes.c_uint8(ord("T")),
#         ctypes.c_uint8(ord("S")),
#         ctypes.c_uint32(4015) # 40.15 degrees * 100
#     )

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # Send the packet
#     serial_port.write(packet_string)

#     # Check for an OK packet
#     read_results = serial_port.read(1024)
#     print(read_results)
#     assert (acknowledgement_packet in read_results), "Oven fan on packet not acknowledged"

#     time.sleep(15)

#     # Create a packet to turn the oven fan off
#     packet_transcoder.packetEncode_8_32(
#         ctypes.byref(packet),
#         ctypes.c_uint8(ord("C")),
#         ctypes.c_uint8(ord("T")),
#         ctypes.c_uint8(ord("Q")),
#         ctypes.c_uint32(0) # 0 degrees * 100
#     )

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # Send the packet
#     serial_port.write(packet_string)

#     # Check for an OK packet
#     read_results = serial_port.read(1024)
#     print(read_results)
#     assert (acknowledgement_packet in read_results), "Oven fan off packet not acknowledged"


# @pytest.mark.skip(reason="Not interesting right now")
# def test_data_acquisition_toggle(serial_port, packet_transcoder, acknowledgement_packet, valves):
#     """
#     Function is responsible for toggling the data acquisition through all of its states
#     """

#     DAQ_states = range(0, 4)

#     # Create a packet to toggle the data acquisition
#     packet = Packet()

#     # Create a packet to change the state of the data acquisition
#     for state in DAQ_states:
#         packet_transcoder.packetEncode_16(
#             ctypes.c_uint8(ord("C")),
#             ctypes.c_uint8(ord("U")),
#             ctypes.byref(packet),
#             ctypes.c_uint16(state),
#         )

#         # Serialize the packet into a hex encoded string
#         packet_string = PROTO_PREFIX
#         packet_string += bytes(packet.raw).hex().upper().encode()
#         print(packet_string.decode())

#         # Send the packet
#         serial_port.write(packet_string)

#         # Check for an OK packet
#         read_results = serial_port.read(1024)
#         print(read_results)
#         assert (
#             acknowledgement_packet in read_results
#         ), "DAQ toggle packet not acknowledged"


@pytest.mark.skip(reason="Not interesting right now")
def test_oven_controller(nodes):
    """
    Function is responsible for testing the PID controller of the oven.
    The PID controller should be able to operate in a range of 20-80 degrees C.
    The PID controller shouldn't overshoot the set point by more than 0.5 degrees C.
    The PID controller should be able to hold the set point +- 0.1 degree C for 1 minute.
    The PID controller should be able to settle in under 10 minutes.
    """
    firmware_node, test_node = nodes

    def float_to_uint32(float_number):
        float_type = ctypes.c_float(float_number)
        int_type = ctypes.cast(ctypes.pointer(float_type), ctypes.POINTER(ctypes.c_uint32))
        return int_type.contents.value

    # Create a blank packet
    packet = Packet()

    # Enable temperature sensor output
    firmware_node.set_data_acquisition_state(DataAcquisitionState.CARTRIDGE_ONLY)
    
    set_point = 40.15
    tolerance = 0.1
    is_settled = False

    # Create a packet to turn the oven fan on
    packet_transcoder.packetEncode_8_32(
        ctypes.byref(packet),
        ctypes.c_uint8(ord("C")),
        ctypes.c_uint8(ord("T")),
        ctypes.c_uint8(ord("S")),
        ctypes.c_uint32(float_to_uint32(set_point * 100)) # 40.15 degrees * 100
    )

    # Serialize the packet into a hex encoded string
    packet_string = PROTO_PREFIX
    packet_string += bytes(packet.raw).hex().upper().encode()
    print(packet_string.decode())

    # Send the packet
    serial_port.write(packet_string)

    # Check for an OK packet
    read_results = serial_port.read(1024)
    print(read_results)
    assert (acknowledgement_packet in read_results), "Oven on packet not acknowledged"

    # Create a deque to store the last 60 temperature readings
    temperature_readings = deque(maxlen=60)

    # Get the current time and limit test to 10 mins
    start_time = time.time()
    while (time.time() - start_time) < 600:
        line = serial_port.readline().strip()
        print(line)
        """
        Parse the data into the packet
        """
        # Split the line on spaces
        split_line = line.split(b" ")
        if split_line[0] == b"proto1":
            # Convert the hex string to bytes
            hex_string = split_line[1].decode()
            hex_bytes = bytes.fromhex(hex_string)

            # Copy the bytes into the packet
            ctypes.memmove(ctypes.byref(packet.raw), hex_bytes, len(hex_bytes))

            # Check if the packet is a temperature data packet
            if packet.field.packet_ID == ord("D") and packet.field.type_ID == ord("T"):

                # Decode the packet
                sequence_number = ctypes.c_uint32(0)
                oven_state = ctypes.c_uint8(0)
                temperature = ctypes.c_uint16(0)
                packet_transcoder.packetDecode_data(
                    ctypes.byref(packet),
                    ctypes.byref(sequence_number),
                    ctypes.byref(oven_state),
                    ctypes.byref(temperature),
                )

                # Convert and insert
                temperature_as_float = temperature.value / 100
                temperature_readings.appendleft(temperature_as_float)

                # Check if the temperature is stabilized
                if all(set_point - tolerance <= item <= set_point + tolerance for item in temperature_readings):
                    is_settled = True
                    break
    
    # Check if the temperature is stabilized
    assert is_settled, "Oven temperature did not stabilize"

    # Turn off the temperature sensor output
    state = 0
    packet_transcoder.packetEncode_16(
        ctypes.c_uint8(ord("C")),
        ctypes.c_uint8(ord("U")),
        ctypes.byref(packet),
        ctypes.c_uint16(state),
    )

    # Serialize the packet into a hex encoded string
    packet_string = PROTO_PREFIX
    packet_string += bytes(packet.raw).hex().upper().encode()
    print(packet_string.decode())

    # Send the packet
    serial_port.write(packet_string)

    # Check for an OK packet
    read_results = serial_port.read(1024)
    print(read_results)
    assert (acknowledgement_packet in read_results), "Data acquisition off packet acknowledged"


    # Wait for the oven to settle
    # Parse the temperature sensor output

    # Create a packet to turn the oven fan off
    packet_transcoder.packetEncode_8_32(
        ctypes.byref(packet),
        ctypes.c_uint8(ord("C")),
        ctypes.c_uint8(ord("T")),
        ctypes.c_uint8(ord("Q")),
        ctypes.c_uint32(0) # 0 degrees * 100
    )

    # Serialize the packet into a hex encoded string
    packet_string = PROTO_PREFIX
    packet_string += bytes(packet.raw).hex().upper().encode()
    print(packet_string.decode())

    # Send the packet
    serial_port.write(packet_string)

    # Check for an OK packet
    read_results = serial_port.read(1024)
    print(read_results)
    assert (acknowledgement_packet in read_results), "Oven off packet not acknowledged"


# def test_oven_step_response(serial_port, packet_transcoder, acknowledgement_packet):
#     """
#     Function is responsible for testing the step response of the oven.
#     It should record a csv file of the step response.
#     """


# @pytest.mark.skip(reason="Not interesting right now")
# def test_method_phase(serial_port, packet_transcoder, acknowledgement_packet):
#     """
#     Function is responsible for checking that the device is able to go through
#     all the required phases
#     """

#     # Create a packet
#     packet = Packet()

#     # Create an empty array of bytes
#     arr = (ctypes.c_int * 13)()

#     # Encode the packet
#     packet_transcoder.packetEncode_text(
#         ctypes.c_uint8(ord("C")),
#         ctypes.c_uint8(ord("N")),
#         ctypes.byref(packet),
#         bytes(arr),
#     )

#     # Serialize the packet into a hex encoded string
#     packet_string = PROTO_PREFIX
#     packet_string += bytes(packet.raw).hex().upper().encode()
#     print(packet_string.decode())

#     # Send the packet
#     serial_port.write(packet_string)

#     # Check for an OK packet
#     read_results = serial_port.read(1024)
#     print(read_results)
#     assert (acknowledgement_packet in read_results), "Device not in configuration phase"


# def test_method(serial_port, packet_transcoder, acknowledgement_packet):
#     """
#     Function is responsible for 
#     """

