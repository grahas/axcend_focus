import pytest
from flask import current_app
import unittest
import rclpy
import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import launch_testing.tools
import launch_ros.actions
from axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface import (
    app,
    system_state,
    LegacyCompatibilityInterface,
)
import axcend_focus_ros2_firmware_bridge.packet_definitions as packet_definitions
from axcend_focus_test_utils_package.conftest import nodes, mock_serial_port
import time
import threading

# Create an instance of the PacketTranscoder
packet_transcoder = packet_definitions.PacketTranscoder()


@pytest.fixture
def client():
    with app.test_client() as client:
        yield client

@pytest.fixture(autouse=True)
def reset_system_state():
    """Reset the system state before and after each test."""
    # Code to reset the system state goes here
    system_state["operator"] = None
    system_state["operator_ip"] = None
    system_state["reservation_key"] = None
    system_state["is_reserved"] = False

    # Yield control to the test function
    yield

    # Code to clean up after the test goes here (if necessary)


@pytest.fixture
def nodes_with_legacy(nodes):
    """This turns the nodes fixture into a fixture that includes the legacy compatibility interface node."""
    # Add the legacy compatibility interface node
    legacy_compatibility_interface = LegacyCompatibilityInterface(
        system_state["firmware_UART_write_queue"],
        system_state["firmware_UART_read_queue"])
    nodes['executor'].add_node(legacy_compatibility_interface)
    
    # Start the publish thread
    publish_thread = threading.Thread(
        target=legacy_compatibility_interface.publish_to_firmware_UART_write_thread
    )
    publish_thread.start()

    yield nodes

    # Clean up the legacy compatibility interface node
    legacy_compatibility_interface.shutdown_event.set()
    publish_thread.join()


def test_home(client):
    """Test the home route."""
    response = client.get("/")
    assert response.status_code == 200
    assert response.data == b"Hello, World!"


def test_is_reserved(client):
    """Test the is_reserved route."""
    response = client.get("/IsReserved")
    assert response.status_code == 200
    assert b"NOT RESERVED" in response.data


def test_reserve_and_release_system(client):
    """Test the reserve_system and release_system routes together."""
    # Test reserve_system route
    response = client.get("/ReserveSystem?user=test_user")
    assert response.status_code == 200
    assert b"RESERVED" in response.data
    test_key = response.json["result"]["key"]

    # Test release_system route
    response = client.get(f"/ReleaseSystem?key={test_key}")
    assert response.status_code == 200
    assert b"OK" in response.data


def test_reserve_system_already_reserved(client):
    """Test the reserve_system route when the system is already reserved."""
    client.get("/ReserveSystem?user=test_user")
    response = client.get("/ReserveSystem?user=test_user")
    assert response.status_code == 200
    assert b"ALREADY RESERVED" in response.data


def test_release_system_invalid_key(client):
    """Test the release_system route with an invalid key."""
    client.get("/ReserveSystem?user=test_user")
    response = client.get("/ReleaseSystem?key=invalid_key")
    assert response.status_code == 200
    assert b"INVALID KEY" in response.data


def test_release_system_not_reserved(client):
    """Test the release_system route when the system is not reserved."""
    response = client.get("/ReleaseSystem?key=test_key")
    assert response.status_code == 200
    assert b"NOT RESERVED" in response.data


def test_get_machine_name(client):
    """Test the machinename route."""
    response = client.get("/machinename")
    assert response.status_code == 200


def test_get_version(client):
    """Test the version route."""
    response = client.get("/version")
    assert response.status_code == 200


def test_read(client, nodes_with_legacy):
    """Test the read route."""
    # Get testing tools
    mock_serial_port = nodes_with_legacy['mock_serial_port']

    # Populate some dummy data for the client to read
    mock_serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())
    mock_serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())
    mock_serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())
    mock_serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())

    # Create the expected response
    expected_response = "".join(packet_transcoder.create_heartbeat_packet().split("proto1 ")[1]*4)
    expected_response = "proto1 " + expected_response

    # Sleep for one second to allow the data to be read
    time.sleep(1)

    # Test the read route
    response = client.get("/read")

    # Check the response
    assert response.status_code == 200
    assert expected_response.encode() in response.data


def test_write(client, nodes_with_legacy):
    """Test the write route."""
    # Get testing tools
    mock_serial_port = nodes_with_legacy['mock_serial_port']

    # Create a dummy packet to write
    dummy_packet = packet_transcoder.create_heartbeat_packet()
    dummy_packet = 'proto1 4748000000000001000000000000000A'

    # Write the packet
    response = client.get(f"/write?data={dummy_packet.split('proto1 ')[1]}")

    # Sleep for one second to allow the data to be written
    time.sleep(1)
    
    # Check the response
    assert response.status_code == 200
    assert b"OK" in response.data

    # Check the data in the firmware_UART_write_queue
    assert dummy_packet.encode() in mock_serial_port.write_data
