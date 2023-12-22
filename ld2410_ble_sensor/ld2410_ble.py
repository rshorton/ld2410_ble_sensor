# This node was derived from the run.py example of the ld2410_ble package (Apache 2.0)
# (https://github.com/930913/ld2410-ble) and the example code from the page
# https://robotics.stackexchange.com/questions/24304/how-to-bridge-rclpy-with-pythons-asyncio
# showing how to create a ROS python node using python asyncio.
#
# This node monitors the output of the LD2410 BLE motion/presence sensor and
# and reports the status using the topic /ld2410/status using the 'Status'
# message of the ld2410_sensor_interfaces package.
#
# This package uses the LD2410_ble package for interface with the sensor using BLE.
#
# TODO: support parameters for configuring the sensor

import asyncio
import threading

from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

import rclpy
from rclpy.node import Node

from ld2410_sensor_interfaces.msg import Status

from ld2410_ble.ld2410_ble import LD2410BLE, LD2410BLEState

# Set the address of your device here.  To determine the address,
# set LIST_ALL_DISCOVERED_BLE_DEVICES=True
# and then look for a device with 'HLK-LD2410' in the listing.
ADDRESS = "1c:b9:65:02:01:94"
RUN_FOREVER = True
LIST_ALL_DISCOVERED_BLE_DEVICES = True

async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)

async def main() -> None:
    node = Node('unused')
    node.get_logger().info('Created LD2410BLE node')

    status_pub = node.create_publisher(Status, '/ld2410/status', 10)

    scanner = BleakScanner()
    future: asyncio.Future[BLEDevice] = asyncio.Future()

    def on_detected(device: BLEDevice, adv: AdvertisementData) -> None:
        if future.done():
            return
        if LIST_ALL_DISCOVERED_BLE_DEVICES:
            node.get_logger().info("Detected: %s, address: %s" % (device, device.address.lower()))
        if device.address.lower() == ADDRESS.lower():
            node.get_logger().info("Found device: %s" % device.address)
            future.set_result(device)

    scanner.register_detection_callback(on_detected)
    await scanner.start()

    def on_state_changed(state: LD2410BLEState) -> None:
        #node.get_logger().info("State changed: %s", state)
        if state.is_moving:
           node.get_logger().info("                                   moving (%0.2f m)" % (state.moving_target_distance/100.0))
        if state.is_static:           
           node.get_logger().info("                 static (%0.2f m)" % (state.static_target_distance/100.0))
        if not state.is_moving and not state.is_static:
           node.get_logger().info("no detection")

        msg = Status()
        if state.is_moving:
            msg.detection_state = Status.DETECTION_MOVING
            msg.distance = state.moving_target_distance/100.0
        elif state.is_static:
            msg.detection_state = Status.DETECTION_STATIC
            msg.distance = state.static_target_distance/100.0
        else:    
            msg.detection_state = Status.DETECTION_NONE
        status_pub.publish(msg)

    device = await future
    await scanner.stop()

    ld2410b = LD2410BLE(device)
    await ld2410b.initialise()

    node.get_logger().info("initial config:")
    node.get_logger().info("motion energy config: %s" % str(ld2410b.config_motion_energy_gates))
    node.get_logger().info("static energy config: %s" % str(ld2410b.config_static_energy_gates))

    # FIX - specify these values using parameters
    # Set the configuration...
    await ld2410b.config_gate_sensitivity(0, 75, 50)    # gate, moving sensitivity, static sensitivity
    await ld2410b.config_gate_sensitivity(1, 75, 50)
    await ld2410b.config_gate_sensitivity(2, 50, 50)
    await ld2410b.config_gate_sensitivity(3, 30, 64)
    await ld2410b.config_gate_sensitivity(4, 15, 64)
    await ld2410b.config_gate_sensitivity(5, 15, 64)
    await ld2410b.config_gate_sensitivity(6, 15, 20)
    await ld2410b.config_gate_sensitivity(7, 15, 20)
    await ld2410b.config_gate_sensitivity(8, 15, 20)
    
    await ld2410b.config_max_gate_and_unmanned_timeout(4, 4, 5) # max moving gate, max static gate, unmanned timeout

    await ld2410b.read_config()

    node.get_logger().info("after configuring:")
    node.get_logger().info("motion energy config: %s" % str(ld2410b.config_motion_energy_gates))
    node.get_logger().info("static energy config: %s" % str(ld2410b.config_static_energy_gates))
    node.get_logger().info("max motion gate: %s" % str(ld2410b.config_max_motion_gates))
    node.get_logger().info("max static gate: %s" % str(ld2410b.config_max_static_gates))
    node.get_logger().info("unmanned timeout: %s sec" % str(ld2410b.config_unmanned_timeout))

    cancel_callback = ld2410b.register_callback(on_state_changed)
    await ld2410b.initialise()

    spin_task = asyncio.get_event_loop().create_task(spin(node))
    await asyncio.wait([spin_task], return_when=asyncio.FIRST_COMPLETED)

    cancel_callback()
    if spin_task.cancel():
        await spin_task

rclpy.init()
asyncio.run(main())
rclpy.shutdown()

if __name__ == '__main__':
    main()
