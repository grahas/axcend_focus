import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evdev import InputDevice, categorize, ecodes, list_devices
import threading

def find_button_devices():
    devices = [InputDevice(path) for path in list_devices()]
    button_devices = []
    for device in devices:
        if "front-panel" in device.name.lower():
            button_devices.append(device)
    return button_devices

class FrontPanelButtonController(Node):
    def __init__(self):
        super().__init__('front_panel_button_controller')
        self.publisher_ = self.create_publisher(String, 'front_panel_button', 10)
        self.button_devices = find_button_devices()
        if self.button_devices:
            self.listen_thread = threading.Thread(target=self.listen_to_buttons)
            self.listen_thread.start()
        else:
            self.get_logger().info("No front panel button devices found.")

    def listen_to_buttons(self):
        button_press_times = {}
        for device in self.button_devices:
            self.get_logger().info(f'Listening on {device.name}...')
            for event in device.read_loop():
                if event.type == ecodes.EV_KEY:
                    key_event = categorize(event)
                    if key_event.keystate == key_event.key_down:
                        # Record the time when the button is pressed
                        button_press_times[key_event.keycode] = time.time()
                    elif key_event.keystate == key_event.key_up:
                        # Calculate the duration of the press
                        if key_event.keycode in button_press_times:
                            press_duration = time.time() - button_press_times[key_event.keycode]
                            button_press_msg = String()
                            button_press_msg.data = f'Button {key_event.keycode} pressed for {press_duration:.2f} seconds'
                            self.publisher_.publish(button_press_msg)
                            self.get_logger().info(button_press_msg.data)
                            # Remove the keycode from the dictionary after calculating the duration
                            del button_press_times[key_event.keycode]

def main(args=None):
    rclpy.init(args=args)
    front_panel_button_controller = FrontPanelButtonController()
    rclpy.spin(front_panel_button_controller)
    front_panel_button_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()