"""
File provides encoding and decoding of packets sent to and from the firmware
"""

import ctypes
from enum import Enum
import os
import json


PACKET_MAX_LENGTH = 16
PROTO_PREFIX = b"proto1 "


class DataAcquisitionState(Enum):
    """Enum is responsible for defining the data acquisition state"""

    DISABLED = 0
    CARTRIDGE_ONLY = 1
    VALVE_PUMPS_ONLY = 2
    CARTRIDGE_AND_VALVE_PUMPS = 3


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


class PacketTranscoder:
    """Class that gives the ability to encode and decode packets"""

    def __init__(self, lib_path: str):
        self.packet_transcoder = self.load_packet_transcoder(lib_path)

    def load_packet_transcoder(self, lib_path: str) -> ctypes.CDLL:
        """
        Load the packet encoding / decoding library and define argument types
        and return types for various functions within the library.
        """
        if not os.path.isfile(lib_path):
            raise FileNotFoundError(f"The library '{lib_path}' does not exist.")

        print("Loading the packet encoding / decoding library")
        packet_transcoder = ctypes.CDLL(lib_path)

        # Define the argument types and return type for each function
        packet_transcoder.packetEncode_0.argtypes = [
            ctypes.c_uint8,  # for uint8_t packet_ID
            ctypes.c_uint8,  # for uint8_t type_ID
            ctypes.POINTER(Packet),  # for Packet *packet
        ]
        packet_transcoder.packetEncode_0.restype = ctypes.c_int  # for int return type

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
            ctypes.POINTER(ctypes.c_uint8),
        ]
        packet_transcoder.packetDecode_oven_data.restype = ctypes.c_int

        return packet_transcoder

    def create_heartbeat_packet(self) -> str:
        """Return the encoded heart beat packet"""
        packet = Packet()

        # Make an OK packet
        packet_ID = ctypes.c_uint8(ord("G"))
        type_ID = ctypes.c_uint8(ord("H"))
        self.packet_transcoder.packetEncode_0(packet_ID, type_ID, ctypes.byref(packet))

        # Serialize the packet into a hex encoded string
        packet_string = "proto1 " + bytes(packet.raw).hex().upper()
        return packet_string

    def create_acknowledgement_packet(self) -> str:
        """Create a default acknowledgement packet hex encoded string"""
        ACK_VALUE = b"OK"

        packet = Packet()

        # Make an OK packet
        packet_ID = ctypes.c_uint8(ord("D"))
        type_ID = ctypes.c_uint8(ord("A"))
        buf = ctypes.create_string_buffer(ACK_VALUE, len(ACK_VALUE))
        self.packet_transcoder.packetEncode_text(
            packet_ID, type_ID, ctypes.byref(packet), buf
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_system_parameters_packet(self) -> str:
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

    def decode_packet(self, packet_string: str) -> list:
        """Decode a hex string packet from the firmware and return the data as a dictionary"""
        # Convert the hex encoded data to byte array
        prefix, data = packet_string.split(" ", 1)

        data = bytes.fromhex(data)

        # Get the packet type
        packet_type = chr(data[0]) + chr(data[1])

        # Check the size of the packet
        if prefix == "proto1":
            # Make sure the packet is the correct size
            assert len(data) == 16, "Invalid packet size for proto1"  # 16 bytes

            # Move the data into a packet object
            packet = Packet()

            # Copy the bytes into the packet
            ctypes.memmove(ctypes.byref(packet.raw), data, len(data))

        elif prefix == "cartridge_config":
            assert (
                len(data) == 84
            ), "Invalid packet size for cartridge_config"  # 84 bytes
            packet_type = "cartridge_config"

            # Move the data into a packet object
            packet = CartridgeMemory_t()

            # Copy the bytes into the packet
            ctypes.memmove(ctypes.byref(packet.raw), data, len(data))

        return [prefix, packet_type, packet]

    def create_valve_rotate_packet(
        self, solvent_valve_position: int, injection_valve_position: int
    ) -> str:
        """Create a rotate valve packet"""
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("C"))
        type_ID = ctypes.c_uint8(ord("V"))
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
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_cartridge_memory_write_packet(self, cartridge_data: dict) -> str:
        """Create a cartridge memory write packet"""
        packet = CartridgeMemory_t()

        # Set the fields of the union according to the data in the JSON object
        packet.data.version = int(cartridge_data["version"])
        packet.data.revision = int(cartridge_data["revision"])
        packet.data.serial_number = cartridge_data["serial_number"].encode()
        packet.data.max_pressure_rating = int(cartridge_data["max_pressure_rating"])
        packet.data.has_oven = int(cartridge_data["has_oven"])
        packet.data.is_third_party = int(cartridge_data["is_third_party"])
        packet.data.last_write_date = cartridge_data["last_write_date"].encode()
        packet.data.total_run_time = int(cartridge_data["total_run_time"])
        packet.data.cartridge_flags = int(cartridge_data["cartridge_flags"])
        packet.data.detector_count = int(cartridge_data["detector_count"])
        packet.data.detector_1_type = int(cartridge_data["detector_1_type"])
        packet.data.detector_1_wavelength = int(cartridge_data["detector_1_wavelength"])
        packet.data.detector_1_path_length = int(
            cartridge_data["detector_1_path_length"]
        )
        packet.data.detector_2_type = int(cartridge_data["detector_2_type"])
        packet.data.detector_2_wavelength = int(cartridge_data["detector_2_wavelength"])
        packet.data.detector_2_path_length = int(
            cartridge_data["detector_2_path_length"]
        )
        packet.data.column_count = int(cartridge_data["column_count"])
        packet.data.column_1_length = int(cartridge_data["column_1_length"])
        packet.data.column_1_diameter = int(cartridge_data["column_1_diameter"])
        packet.data.column_1_particle_size = int(
            cartridge_data["column_1_particle_size"]
        )
        packet.data.column_1_description = cartridge_data[
            "column_1_description"
        ].encode()
        packet.data.column_2_length = int(cartridge_data["column_2_length"])
        packet.data.column_2_diameter = int(cartridge_data["column_2_diameter"])
        packet.data.column_2_particle_size = int(
            cartridge_data["column_2_particle_size"]
        )
        packet.data.column_2_description = cartridge_data[
            "column_2_description"
        ].encode()
        packet.data.detector_1_adc_gain = int(cartridge_data["detector_1_adc_gain"])
        packet.data.detector_1_adc_delay = int(cartridge_data["detector_1_adc_delay"])
        packet.data.detector_1_led_pot = int(cartridge_data["detector_1_led_pot"])
        packet.data.detector_2_adc_gain = int(cartridge_data["detector_2_adc_gain"])
        packet.data.detector_2_adc_delay = int(cartridge_data["detector_2_adc_delay"])
        packet.data.detector_2_led_pot = int(cartridge_data["detector_2_led_pot"])
        packet.data.bluetooth_disabled = int(cartridge_data["bluetooth_disabled"])
        packet.data.report_rate = int(cartridge_data["report_rate"])
        packet.data.word_rate = int(cartridge_data["word_rate"])
        packet.data.unused = int(cartridge_data["unused"])
        packet.data.samples_per_point = int(cartridge_data["samples_per_point"])
        packet.data.bits_to_remove = int(cartridge_data["bits_to_remove"])

        # Serialize the struct into a hex encoded string
        packet_string = b"cartridge_config "
        packet_string += bytes(packet.raw).hex().upper().encode()

        return packet_string

    def parse_values_oven_status_packet(self, packet: Packet) -> list:
        """Parse the values from the oven packet"""
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

        return [sequence_number, oven_state, temperature_as_float, power_output]

    def create_data_acquisition_state_packet(self, state: DataAcquisitionState) -> str:
        """Create a data acquisition state packet"""
        # Create a blank packet
        packet = Packet()

        # Enable temperature sensor output
        self.packet_transcoder.packetEncode_16(
            ctypes.c_uint8(ord("C")),
            ctypes.c_uint8(ord("U")),
            ctypes.byref(packet),
            ctypes.c_uint16(state.value),
        )

        return PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
