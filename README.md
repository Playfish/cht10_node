# cht10_node

## Description
This ROS package is Driver for CHT-10 laser ranging sensor of LAX.

### Published Topics

 */range (sensor_msgs/Range)
  *Provides range reading.

### Parameters

 *serialNumber (str, default:"/dev/ttyUSB0")
  *The device path to open. 

 *frame_id (str, default:"laser")
  *Frame id for sensor.

 *baudRate (int, default:"115200")
  *baud rate for sensor.

## Compile

For compile this package you should in your ``catkin_ws``, and typing:

```
cd <your_catkin_ws>/src
git clone https://github.com/Playfish/cht10_node
cd ..
catkin_make
```

## Usage

For using this package you should enter following command:
```
roslaunch cht10_node standalone.launch
```

For view typing:
```
roslaunch cht10_node view_range.launch
```
