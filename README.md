# A modified version of the original ROS flir_one_node package: https://github.com/aplyer/flir_one_node 

install udev rules:
  >sudo cp 51-usb-flir-one.rules /etc/udev/rules.d

the node publish : 
- RGB stream
- IR stream

check the provided launch file. It has the following parameters:
 - min_temp [celsius].- temperature that corresponds to pure blue pixel value. Any temp below this one will be represented in blue
 - max_temp [celsius].- temperature that corresponds to pure red pixel value. Any temp above this one will be represented in red
 - publish_rgb_image.- if true, RGB image will be generated, but this consumes more CPU. If you don't really need it, put false
 - publish_ir_image.- if true, IR image will be generated, this doesn't make too much difference in CPU consumption but you can set it to false if you are only going to use the colour image







