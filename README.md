# ros2pkg_configure_nodejs
**ros2pkg_configure_nodejs** is an extension, i.e., plugin, for the `ros2 pkg` CLI command that simplifies the implementation of ROS2 packages in JavaScript or TypeScript. The plugin extends an existing ROS2 package by scaffolding onto it customized Node.js artifacts, examples and updated installation scripts. **The result is a ROS2-Node.js package that can coexist and participate with other packages in a ROS2 workspace and can be run using the ROS2 `launch` facility.**

## Key Features
* Customizes and overlays Node.js resources onto ROS2 ament_cmake and cmake packages.
* Creates a package.json based on ROS2 package.xml Manifest
* `--typescript` commandline option to configure the package for use with TypeScript.
* Includes the ROS2 JavaScript client, 
[rclnodejs](https://github.com/RobotWebTools/rclnodejs) as a runtime dependency.
* Creates an example launch-description.
* Updates CMakeList.txt install() rules to install key runtime files to the package share/ folder
* Example JavaScript/TypeScript ROS2 publisher node.

## Prerequisites
* ROS2 Foxy or greater installed
* Node.js version 12 or 13
* npm or yarn package manager

## Getting Started

### 1. Clone and Build ros2pkg_configure_nodejs
This is a new ROS2 package and not yet part of a formal ROS2 distribution. At some point in the future I hope to see that happen. Until then you will need to clone and build this GIT repository - a 1 minute task.

From a command shell `cd` into a directory that will contain this package. Then enter the following commands:
```
git clone https://github.com/ros2jsguy/ros2pkg_configure_nodejs.git
cd ros2pkg_configure_nodejs
colcon build
```
Next let's add the newly built ros2pkg_configure_nodejs package to our ROS2 environment. Do this by running the `install/setup.[bash|bat|sh|ps1]` file for your environment. For background on configuring your ROS2 environement see this [tutorial](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/).

In my Linux environment I run this command:
```
source install/setup.bash
```
For future convience consider including the setup.[bat|bash|ps1] in your login script.

Let's verify that ros2pkg_configure_nodejs is installed properly:
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

### 2. Create ROS2 Package and Configure it as a Node.js Package
In this section we will create a basic ROS2 package and then configuring it as a Node.js package.

From your command shell `cd` to a directory that will contain the ROS2 package we will create. 

For this tutorial we will name our ROS2 package `ros2_nodejs`. See [ROS2 Patterns and Conventions](http://wiki.ros.org/ROS/Patterns/Conventions) for naming practices.

Enter the following commands:
```
ros2 pkg create ros2_nodejs
cd ros2_nodejs
ros2 pkg configure_nodejs
```
The ros2_nodejs directory content should be similar to this listing.
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

Now let's build the package using the `colcon` build utility. This will install the key JavaScript resources into the share/ folder.
```
colcon build
```
Your package folder will now include the standard ROS2 package directories: `build/`, `install/` and `log/`. The `install/' directory includes configuration scripts and if you look deep to `install/ros2_nodejs/share/ros2_nodejs` you will see the `install()` rules added to the CMakeLists.txt have installed the key JavaScript resources from the `src/` and `launch/` directories.

Lastly, add your new ros2_nodejs package to your ROS environment. 

From your command shell enter:
```
source install/setup.bash
```
Verify that the ros2_nodejs package is part of your ROS2 environment using the `ros2` CLI command. 
```
ros2 pkg list
```
This command will output a long list of the packages in your ROS2 environment. Scroll through the list and verify it contains the ros2_nodejs package.

### 3. Launch example.launch.py
We can now use the `ros2 launch` command to run the `example.launch.py` launch-description. This launch file defines how to startup the example app in our ros2_nodejs package. The example app creates a ROS2 node and publisher that sends a message every second to the topic named `foo`. See `src/index.js` in the ros2_nodejs package for details of the JavaScript implementation.
```
ros2 launch ros2_nodejs example.launch.py
```
To view the messages being published to the `foo` topic, open a separate command shell configured with your ROS2 environment and enter:
```
ros2 topic echo foo
```
A message should appear every second.

## Working With Typescript
If you would like to work with TypeScript instead of JavaScript use the `--typescript` commandline option as shown below.
```
ros2 pkg configure_nodejs --typescript
```
The plugin will include a tsconfig.json file and a TypeScript example at `src/index.ts`.

# Getting Help / Providing Feedback
Please post bug reports, feature requests and general discussion topics to the [ros2pkg_configure_nodejs project on github](https://github.com/ros2jsguy/ros2pkg_configure_nodejs).

# Thanks
A special thanks to the rclnodejs team for developing the [rclnodejs, the ROS2 JavaScript SDK](https://github.com/RobotWebTools/rclnodejs) and supporting JavaScript in robotics.



