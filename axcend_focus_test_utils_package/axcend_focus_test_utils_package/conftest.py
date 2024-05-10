from unittest.mock import Mock

import pytest
import os
import time
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Temperature
from threading import Thread
from serial import SerialTimeoutException

from axcend_focus_ros2_firmware_bridge.firmware_manager import (
    FirmwareNode,
    DataAcquisitionState,
)
from axcend_focus_custom_interfaces.srv import CartridgeMemoryReadWrite


# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the relative path to the JSON file
relative_path = "dummy_system_parameter_file.json"

# Join the current directory with the relative path
json_file_path = os.path.join(current_dir, relative_path)

# Set the SYS_PARAMS_FILE environment variable
os.environ["SYS_PARAMS_FILE"] = json_file_path


class ExampleNode(Node):
    """Node is for use of the firmware test script."""

    def __init__(self):
        super().__init__("test_firmware_node")
        self.subscription = self.create_subscription(
            Temperature, "cartridge_temperature", self.listener_callback, 10
        )
        self.msg_data = None
        self.cartridge_temperature = deque(maxlen=10)

        # Create a client for the cartridge_memory_read_write service
        self.cartridge_memory_read_write_client = self.create_client(
            CartridgeMemoryReadWrite, "cartridge_memory_read_write"
        )

        # Create a publisher to write to the firmware_UART_write string topic
        self.firmware_UART_write_publisher = self.create_publisher(
            String, "firmware_UART_write", 10
        )

    def listener_callback(self, msg):
        """Callback function for the cartridge temperature subscriber."""
        self.cartridge_temperature.append(msg.temperature)

    def request_cartridge_memory(self):
        """Send a request to the cartridge_memory_read_write service."""
        request = CartridgeMemoryReadWrite.Request()
        request.command = "read"
        while not self.cartridge_memory_read_write_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        future = self.cartridge_memory_read_write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed!")
            return None


@pytest.fixture
def mock_serial_port():
    """Mock the serial port for testing."""
    mock_port = Mock()

    mock_port.write_data = []
    mock_port.read_data = []
    mock_port.start_time = None

    def mock_readline():
        """Read data from the serial read buffer."""
        if not mock_port.read_data:
            if mock_port.start_time is None:
                mock_port.start_time = time.time()
            elif time.time() - mock_port.start_time > 0.5:
                raise SerialTimeoutException
            return b""
        else:
            mock_port.start_time = None
            return mock_port.read_data.pop(0)

    def mock_write(data):
        """Add data to the serial write buffer."""
        mock_port.write_data.append(data)

    def add_to_read_buffer(data):
        """Add data to the serial read buffer."""
        mock_port.read_data.append(data)

    mock_port.readline = mock_readline
    mock_port.write = mock_write
    mock_port.close = Mock()  # Add a close method
    mock_port.add_to_read_buffer = add_to_read_buffer

    yield mock_port


@pytest.fixture
def nodes(mock_serial_port):
    """Create the firmware node and the test node."""
    # Start ROS 2 client library
    rclpy.init()

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()

    # Add nodes to executor
    firmware_node = FirmwareNode(serial_port=mock_serial_port)
    test_node = ExampleNode()
    executor.add_node(firmware_node)
    executor.add_node(test_node)

    # Spin in a separate thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    yield {
        "mock_serial_port": mock_serial_port,
        "test_node": test_node,
        "firmware_node": firmware_node,
        "executor": executor,
    }

    # Ensure resources are cleaned up properly
    firmware_node.close()
    rclpy.shutdown()
    executor_thread.join()
