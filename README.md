# ld2410_ble_sensor

ROS2 driver package for the LD2410 BLE human presence sensor.

The node for this package was derived from the run.py example of the ld2410_ble python package
(https://github.com/930913/ld2410-ble) and the example code from the page
https://robotics.stackexchange.com/questions/24304/how-to-bridge-rclpy-with-pythons-asyncio
which shows how to create a ROS2 python node compatible with python asyncio.

The node monitors the LD2410 BLE motion/presence sensor 
and publishes the output using the Status message of the ld2410_sensor_interface package to the topic /ld2410/status.

## Dependencies

ld2410-ble fork: https://github.com/rshorton/ld2410-ble

## Running

```
ros2 run ld2410_ble_sensor ld2410_ble
```

## TODO

Support parameters for configuring the sensor
