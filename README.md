# cht10_node

## Description
This ROS2 package is Driver for CHT-10 laser ranging sensor of LAX.

### Published Topics

 * /range (sensor_msgs/Range)
   * Provides range reading.

### Parameters

 * serialNumber (str, default:"/dev/ttyUSB0")
   * The device path to open. 

 * frame_id (str, default:"laser")
   * Frame id for sensor.

 * baudRate (int, default:"115200")
   * baud rate for sensor.

## Compile

For compile this package you should in your ``ros2_ws``, and typing:

```
cd <ros2_ws>/src
git clone https://github.com/Playfish/cht10_node
cd cht10_node
git checkout ros2
cd ..
ament build src/cht10_node
```

## Usage

For using this package you should enter following command:
```
ros2 run cht10_node cht10_node_ros
```

