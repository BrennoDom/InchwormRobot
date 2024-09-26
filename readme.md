Clone this repo:

```shell
git clone https://github.com/BrennoDom/InchwormRobot.git
```

Connects all motor like the eletrical project below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/2ebd052f-e0d9-49d6-bf6f-b7628a4e63bb" width="500">
</p>

Configure baudrate and id of each motor before with DynamixelWizard. All motors needs have the same value in baudrate.

In file ```/src/escalador_description_serial_2/urdf/escalador_serial_2_ros2_control.xacro``` puts ```baud_rate``` and ```usb_port``` correctly, also needs comment the ```use_dummy``` in description file.


```xml
      <hardware>
         <plugin>dynamixel_hardware/DynamixelHardware</plugin>
         <param name="usb_port">/dev/ttyUSB0</param>
         <param name="baud_rate">1000000</param> 
         <!--<param name="use_dummy">true</param>-->
         <param name="IOInputs">16</param>
         <param name="IOutputs">16</param>
      </hardware>
```
Also, configure the ```offset```,```PID``` gains and ```id``` parameters for each motors like my ```J1```:
```xml
    <joint name="J1">
        <param name="id">01</param>
        <param name="offset">2.39</param>
        <param name="Profile_Velocity">20</param>
        <param name="Position_P_Gain">1200</param>
        <param name="Position_I_Gain">2100</param>
        <param name="Position_D_Gain">8000</param>
        <param name="Feedforward_1st_Gain">0</param>
        <param name="Feedforward_2nd_Gain">0</param>

        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
```



To compile the workspace, you must have ROS2 humble installed on PC and must import the dynamixel_control.repos repo with the command below:


```shell
vcs import src < src/dynamixel_control.repos
```

After compiled and sourced launch with:

```shell
ros2 launch escalador_deploy deploy.launch.py
```
