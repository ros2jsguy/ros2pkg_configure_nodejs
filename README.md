# ros2pkg_configure_nodejs
**ros2pkg_configure_nodejs** is an extension for the `ros2 pkg` cli command.
The plugin configures a ROS2 package as a Node.js package. When used 
with the ROS2 JavaScript client, 
[rclnodejs](https://github.com/RobotWebTools/rclnodejs), it simplifies creation
of ROS2 packages implemented in JavaScript/TypeScript as Node.js packages. 

## Key Features
* Customizes and overlays Node.js resources onto a ROS2 ament_cmake or cmake package.
* Creates package.json based on ROS2 package.xml Manifest
* `--typescript` commandline option to configure the package for use with TypeScript.
* Includes rclnodejs as a runtime dependency.
* Creates example launch-description.
* Updates CMakeList.txt install() rules to copy key runtime files into the package share/ folder
* Example JavaScript/TypeScript ROS2 publisher node.

## Prerequisites
* ROS2 Foxy or greater installed
* Node.js version 12 or 13
* npm or yarn package manager

## Getting Started

### 1 - Clone and Build ros2pkg_configure_nodejs
This is a new package. At some point in the future I hope to see this ROS2 package included in
a ROS distribution. In the meantime you will need to clone and build this GIT repository - a 1 minute task.

From a command shell enter the following commands:
```
cd <parent-directory-of-ros2pkg_configure_nodejs>
git clone https://github.com/ros2jsguy/ros2pkg_configure_nodejs.git
cd ros2pkg_configure_nodejs
colcon build
source install/local_setup.bash # or choose the equivalent for your shell env
```
For future convience consider including the local_setup.[bat|bash|ps1] in your login script.

Verify that ros2pkg_configure_nodejs is installed properly:
Enter this command and observe the output.
```
ros2 pkg -h
```
You should see `configure_nodejs` in the Commands list similar to the output shown below.
```
usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...

Various package related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  configure_nodejs  Configure ROS2 package as Node.js package
  create       Create a new ROS2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package
  xml          Output the XML of the package manifest or a specific tag

  Call `ros2 pkg <command> -h` for more detailed usage.
```

### 2 Create ROS2 Package and Configure it as a Node.js Package
The two step process involves creating a ROS2 package and then configuring it
as a Node.js package.

From a command shell enter:
```
ros2 pkg create <your_pkg_name> 
cd <your_pkg_name>
ros2 pkg configure_nodejs
```
Your ROS2 package contents should be similar to this listing.
```
CMakeLists.txt
__init__.py
jsconfig.json
launch/
  example.launch.py
node_modules/
package.json
package.xml
src/
  index.js
```

Now let's build the package using the `colcon` build utility. This will
install the key JavaScript resources in the share/ folder.
```
cd <your_pkg_name>
colcon build
```
And finally add <your_pkg> to your ROS environment.
```
source install/local_setup.bash
```

### Launch example.launch.py
We can now use the `ros2 launch` command to startup our ROS2 package running
a small Node.js example app that publishes a message every second to the `foo` topic.
See `src/index.js` for details of JavaScript implementation.
```
ros2 launch <your_pkg_name> example.launch.py
```

