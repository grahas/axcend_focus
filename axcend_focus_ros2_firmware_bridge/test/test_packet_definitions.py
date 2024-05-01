import pytest
import platform
import axcend_focus_ros2_firmware_bridge.packet_definitions as packet_definitions


def test_packet_encoding():
    """
    Verify that the packet encoding library is working correctly
    """
    # Check the platform
    if platform.system() == "Windows":
        library_path = "packets.dll"
    else:
        library_path = "packets.so"

    packet_transcoder = packet_definitions.PacketTranscoder(library_path)

    assert (
        packet_transcoder.create_heartbeat_packet()
        == b"proto1 4748000000000000000000000000000A"
    ), "Heartbeat packet not created correctly"
