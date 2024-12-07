# Repository for ROS2 and Axcend Focus LC

## Workflow for building ros2 package and getting them on the board

### Setup

### Open the IDE
This is so that the vscode enviroment process is forked with the correct enviroment variables for the ROS2 workspace and will pass
it on to any tests or other processes so that intelisense and other stuff works correctly

### Linux
Open the Ubuntu WSL VM and build server VM
cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

### Windows
Open powershell and run the following commands
C:\Users\gupyf\Documents\GitHub\ros2_ws\install\setup.ps1
& "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\devenv.exe" "C:\Users\gupyf\Documents\GitHub\ros2_ws"


### Clean Workspace

### Linux
1. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
2. rm -r build install log
3. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
4. code .

### Windows
1. cd C:\Users\gupyf\Documents\GitHub\ros2_ws
2. Remove-Item -Recurse -Force build, install, log

### Dependencies
The code has a dependency that the libaxcend_packet.so is availible on the shared library search path. The library can be compiled by running the make command in the firmware repo and then you can add it to your system. 

### Open the IDE
This is so that the vscode enviroment process is forked with the correct enviroment variables for the ROS2 workspace and will pass 
it on to any tests or other processes so that intelisense and other stuff works correctly
Open Ubuntu WSL terminal
cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
source install/setup.bash
cd src/axcend_focus
code .

### Build Package
1. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
2. colcon build --packages-select axcend_focus_custom_interfaces
3. colcon build --packages-select axcend_focus_front_panel_button

### Source Workspace
1. source install/setup.bash 

### Update Changelog
0. cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
1. catkin_generate_changelog
2. Commit the files
3. catkin_prepare_release
4. y
5. y
6. bloom-release --rosdistro humble axcend_focus
8. https://github.com/grahas/axcend_focus-release.git
9. y
10. y
11. n
12. n
13. n
14. n

https://github.com/grahas/axcend_focus.git

### Update changes in rosdep/python.yaml
rosdep update --include-eol-distros

### Rebuild Package Index

In build-server VM ->

0. In the rosdistro workspace in the VM
1. export ROSDISTRO_INDEX_URL=file:///home/axcend/Documents/GitHub/rosdistro/index-v4.yaml
2. update the version number in distribution.yaml
3. cd ~/Documents/GitHub/rosdistro/humble
4. rosdistro_build_cache file:///home/axcend/Documents/GitHub/rosdistro/index-v4.yaml humble

### Generate the Recipe

In build-server VM ->
0. In the same terminal as before
1. superflore-gen-oe-recipes --dry-run --ros-distro humble --only axcend_focus_custom_interfaces axcend_focus_device_config axcend_focus_front_panel_button axcend_focus_launch axcend_focus_legacy_compatibility_layer axcend_focus_operation axcend_focus_ros2_firmware_bridge axcend_focus_test_utils_package --output-repository-path ~/Documents/GitHub/test-meta-ros


### Update Existing Recipe

1. rm -r /home/axcend/OSTL-k/layers/meta-ros/meta-ros2-humble/generated-recipes/axcend-focus && cp -r /home/axcend/Documents/GitHub/test-meta-ros/meta-ros2-humble/generated-recipes/axcend-focus /home/axcend/OSTL-k/layers/meta-ros/meta-ros2-humble/generated-recipes/axcend-focus
2. update bbappend version number

### Bake Changes

1. bitbake axcend-focus-custom-interfaces
3. bitbake axcend-focus-front-panel-button
4. bitbake axcend-focus-launch
5. bitbake axcend-focus-legacy-compatibility-layer
6. bitbake axcend-focus-ros2-firmware-bridge
7. bitbake axcend-focus-test-utils-package

### Git repo
1. Change git repo back to private

### Copy Changes to Board

1. Update version number if following command
2. To find the package name use: find /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb -name axcend* 
4. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-custom-interfaces_3.1.3-1-r0.0_armhf.deb root@192.168.1.121:/tmp
5. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-legacy-compatibility-layer_3.1.3-1-r0.0_armhf.deb root@192.168.1.121:/tmp
6. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-ros2-firmware-bridge_3.1.3-1-r0.0_armhf.deb root@192.168.1.121:/tmp
7. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-packet-library_1.0+git0+2c549a3a48-r0.4_armhf.deb root@192.168.1.121:/tmp
8. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-launch_3.1.3-1-r0.0_armhf.deb root@192.168.1.121:/tmp
9. scp /home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/tmp-glibc/deploy/deb/cortexa7t2hf-neon-vfpv4/axcend-focus-test-utils-package_3.1.3-1-r0.0_armhf.deb root@192.168.1.121:/tmp
10. 

### Install Changes

1. Update version number in following command
2. dpkg -i /tmp/axcend-focus-custom-interfaces_3.0.8-1-r0.0_armhf.deb

## Create a new package

cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
Examples:
the axcend_focus prefix is for package organization on superflore

ros2 pkg create --build-type ament_cmake --node-name firmware_bridge axcend_focus_ros2_firmware_bridge_cpp
ros2 pkg create --build-type ament_python --node-name front_panel_button_controller axcend_focus_front_panel_button
ros2 pkg create --build-type ament_python --node-name legacy_compatibility_interface axcend_focus_legacy_compatibility_layer
ros2 pkg create --build-type ament_python --node-name system_data_logger axcend_focus_system_data_logger
ros2 pkg create --build-type ament_python axcend_focus_launch
ros2 pkg create --build-type ament_python --node-name device_config axcend_focus_device_config
ros2 pkg create --build-type ament_python --node-name operation axcend_focus_operation
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name axcend_focus_client axcend_focus_client_library_cpp

ros2 pkg create --build-type ament_cmake --node-name my_node my_package


## Set the URL for superflore build

git remote set-url origin https://github.com/grahas/axcend_focus

Set for gitlab
git remote set-url origin https://gitlab.com/axcend/v3-hw-and-sw/axcend_focus

# Map network drive to board

\\sshfs\root@192.168.7.1\..\..
password is root

rsync -avz --exclude '.git/' /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_ros2_firmware_bridge root@192.168.7.1:/axcend/axcend_focus_ros2_firmware_bridge/

From WSL ros2 environment
colcon test --packages-select axcend_focus_legacy_compatibility_layer --output-on-failure

# Install package dependencies

After adding them to the package.xml file
In WSL ROS2 environment
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Test a specific command

test the legacy_compatibility_layer: 
colcon test --packages-select axcend_focus_legacy_compatibility_layer --event-handlers console_direct+

test the packet transcoder:    
colcon test --packages-select axcend_focus_ros2_firmware_bridge --event-handlers console_direct+

To test without rebuilding in ROS2 you can  use
colcon build --symlink-install

# Build a specific package
### Linux:
colcon build --packages-select axcend_focus_ros2_firmware_bridge --symlink-install
colcon build --packages-select axcend_focus_custom_interfaces --symlink-install
colcon build --packages-select axcend_focus_legacy_compatibility_layer --symlink-install
colcon build --packages-select axcend_focus_launch --symlink-install
colcon build --packages-select axcend_focus_test_utils --symlink-install
colcon build --packages-select axcend_focus_device_config --symlink-install
colcon build --packages-select axcend_focus_client_library_cpp --symlink-install

colcon build --symlink-install

### Windows:
colcon build  --symlink-install --merge-install --cmake-args -DPYTHON_EXECUTABLE='C:\Python38\python.exe' -DPython3_ROOT_DIR='C:\Python38' -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_REGISTRY=NEVER

# Launch the nodes
export ENVIRONMENT=development
export ENVIRONMENT=production
ros2 launch axcend_focus_launch application_launch.py
ros2 run axcend_focus_ros2_firmware_bridge firmware_bridge.py
ros2 run axcend_focus_front_panel_button front_panel_button_controller.py
ros2 run axcend_focus_client_library_cpp axcend_focus_client

# Opening the workspace
open from the axcend_focus directory that has all the packages as the root of the workspace.
cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws
source install/setup.bash
cd /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus
code .
This will make all the paths work

# Copy changes to the board

scp -o StrictHostKeyChecking=no /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_ros2_firmware_bridge/axcend_focus_ros2_firmware_bridge/packet_definitions.py root@192.168.7.1:/usr/lib/python3.10/site-packages/axcend_focus_ros2_firmware_bridge/packet_definitions.py

scp -o StrictHostKeyChecking=no /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_legacy_compatibility_layer/axcend_focus_legacy_compatibility_layer/legacy_compatibility_interface.py root@192.168.7.1:/usr/lib/python3.10/site-packages/axcend_focus_legacy_compatibility_layer/legacy_compatibility_interface.py

scp /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_legacy_compatibility_layer/axcend_focus_legacy_compatibility_layer/legacy_compatibility_interface_node.py root@192.168.7.1:/usr/lib/python3.10/site-packages/axcend_focus_legacy_compatibility_layer/legacy_compatibility_interface_node.py

scp -o StrictHostKeyChecking=no /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_ros2_firmware_bridge/axcend_focus_ros2_firmware_bridge/firmware_bridge.py root@192.168.7.1:/usr/lib/python3.10/site-packages/axcend_focus_ros2_firmware_bridge/firmware_bridge.py

scp -o StrictHostKeyChecking=no /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_front_panel_button/axcend_focus_front_panel_button/front_panel_button_controller.py root@192.168.7.1:/home/root/front_panel_button_controller.py

scp -o StrictHostKeyChecking=no /mnt/c/Users/gupyf/Documents/GitHub/ros2_ws/src/axcend_focus/axcend_focus_launch/launch/application_launch.py root@192.168.0.108:/usr/share/axcend_focus_launch/launch/application_launch.py

## ROS2 commands
Fill / Empty to a specific location
This will fill pumps to 70uL
ros2 action send_goal /pump_positioning axcend_focus_custom_interfaces/action/PumpPositioning "{volume: [70, 70]}"

List all the running nodes
ros2 node list

Show all avalible actions
ros2 action list

Move the valves to fill - load
ros2 action send_goal /valve_rotate axcend_focus_custom_interfaces/action/ValveRotate "{valve_position: [0, 1]}"

Moves the valve to block - load
ros2 action send_goal /valve_rotate axcend_focus_custom_interfaces/action/ValveRotate "{valve_position: [1, 1]}"

Moves the valve to fill - inject
ros2 action send_goal /valve_rotate axcend_focus_custom_interfaces/action/ValveRotate "{valve_position: [1, 0]}"

Launch the rosbridge_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

ros2 action send_goal /valve_rotate axcend_focus_custom_interfaces/action/ValveRotate "{valve_position: [1, 2]}"

ros2 launch axcend_focus_launch application_launch.py

## 
Set the board time from the local computer
ssh root@192.168.7.1 "date --set=\"$(date -u '+%Y-%m-%d %H:%M:%S')\" && hwclock --systohc --utc"

systemctl restart application_launch

./scripts/sstate-cache-management.sh --cache-dir=tmp/sstate-cache --stamps-dir=tmp/stamps --remove-unreferenced
./scripts/sstate-cache-management.sh --remove-duplicated -d --cache-dir=/home/axcend/OSTL-k/build-openstlinuxweston-stm32mp1/sstate-cache 


Getting ROS2 working on windows 11
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
add to the power shell file $env:RMW_IMPLEMENTATION='rmw_fastrtps_cpp'

ros2 daemon stop
ros2 daemon start

$env:VERBOSE = 1
colcon build  --packages-select axcend_focus_client_library_cpp --symlink-install --merge-install --event-handlers console_direct+ --cmake-args -DPYTHON_EXECUTABLE='C:\Python38\python.exe' -DPython3_ROOT_DIR='C:\Python38' -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_REGISTRY=NEVER -DCMAKE_BUILD_TYPE=Debug
colcon build  --symlink-install --merge-install --event-handlers console_direct+ --cmake-args -DPYTHON_EXECUTABLE='C:\Python38\python.exe' -DPython3_ROOT_DIR='C:\Python38' -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_REGISTRY=NEVER -DCMAKE_BUILD_TYPE=Debug

colcon test --merge-install

# ROS2 Humble on Windows 11 Visual Studio 2022 notes
## Issue: 
Can't run the the local_setup.ps1 file in powershell on startup. Error message is that it is not digitally signed.
## Solution: 
Find the file ie C:\dev\ros2_humble_debug\local_setup.ps1 right click->properties->unblock or 
```powershell
Unblock-File -Path "C:\dev\ros2_humble_debug\local_setup.ps1"
Unblock-File -Path "C:\dev\ros2_humble\local_setup.ps1"
```
---
## Issue:
Visual Studio and other tools are not using the 3.8 python interpreter and using something too new for ROS2
## Solution:
Configure visual studio to use the python 3.8 interpreter if newer ones are installed. In the CMakeSettings.json file add the following to the CMake command arguments
```Cmake
-DPYTHON_EXECUTABLE='C:\Python38\python.exe' -DPython3_ROOT_DIR='C:\Python38' -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_REGISTRY=NEVER
```
---
## Issue:
ROS2 humble installs the wrong version of the python packages for empy. I think the root of this is that rosdep isn't availible for Windows
```powershell
CMake Error at C:/dev/ros2_humble/share/rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake:59 (message):
...
 AttributeError processing template 'msg.idl.em'

  Traceback (most recent call last):

    File "C:\dev\ros2_humble\Lib\site-packages\rosidl_adapter\resource\__init__.py", line 51, in evaluate_template
      em.BUFFERED_OPT: True,

  AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
```
## Solution:
Install the correct version of the python packages. In my case empy 4.2 was installed and I needed 3.3.4.
```powershell
C:\Python38\python.exe -m pip install empy==3.3.4
```
---
## Issue: 
Numpy is having issues when used with the debug version of ROS2
## Solution:

# Setup ROS2 on Windows 11
## Build virtual enviroment for ROS2 to avoid conflicts with the system python
```powershell
pip install virtualenv
virtualenv --python="C:\Python38\python.exe" "C:\Users\gupyf\Documents\GitHub\ros2_ws\.venv"
C:\Users\gupyf\Documents\GitHub\ros2_ws\.venv\Scripts\activate
```

call C:\dev\ros2_humble\local_setup.bat
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener

cd C:\Users\gupyf\Documents\GitHub\ros2_ws
.\install\local_setup.ps1

colcon build --merge-install --event-handlers console_direct+ --cmake-args -DPYTHON_EXECUTABLE='C:\Users\gupyf\Documents\GitHub\ros2_ws\.venv\Scripts\python.exe' -DPython3_ROOT_DIR='C:\Users\gupyf\Documents\GitHub\ros2_ws\.venv' -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_REGISTRY=NEVER

## Issue:
Error message: "This version of PyQt5.sip requires Python v3.9 or later"
## Solution:
Restrict the version of PyQt5 to 5.15.6
```powershell
python -m pip install -U catkin_pkg cryptography empy==3.3.4 importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5==5.15.6 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro
```


"%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64