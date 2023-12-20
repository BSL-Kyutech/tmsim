# tmsim

**T**ensegrity **M**anipulator **SIM**ulator
![top](https://github.com/BSL-Kyutech/tmsim/assets/9003707/4745c2c0-5bb5-4df6-b129-0ea4d9e00903)

## How to get started

1. Install the Unity Hub (https://unity.com/). You can leave Unity Editor not installed. The installation of the adequate version will be recommended at the first launch of the project.
1. Clone this repo. For Win11 user: You can use git CLI of WSL2 but don't clone into the WSL2 volume. Move to the Win11 original volume (ex: `/mnt/c/...`) and clone it.
1. Download the latest release of ros2-for-unity. The standalone version is easier to use and recommended.
1. *The installation procedure may be close to being updated. Check* (https://github.com/RobotecAI/ros2-for-unity/pull/95). Extract the ZIP file and copy the `Ros2ForUnity` directory into `tmsim/Assets` directory. For Ubuntu user: Make sure `libspdlog-dev` and `libtinyxml2-dev` installed. This issue can be found here: (https://github.com/RobotecAI/ros2-for-unity/issues/78).
1. Launch the Unity Hub installed. Click `Projects` on the left side pane and `Open` on the right. Choose the `tmsim` directory to open. Click the `tmsim` project in the list to launch. The first launch will take some time due to Unity Editor installation, etc. 
1. Let's visit the sample scene! In the project window of Unity Editor, go into the `Scenes` directory in the `Asset` directory. Double click `Assets/Scenes/SampleScene.unity`. Click the `Play` icon on the top.
1. You will see a tensegrity manipulator spawned. The `ROS2 version`, `Build type`, and `RMW` will be shown on the bottom if the ROS2 interface is successfully launched. You can access to the tensegrity manipulator through ROS2 topics.
1. Do you want to see the tensegrity manipulator moving for now? Select the `ctrl` in the `Hierarchy` window and Visit the `Demo (Script)` component in the `Inspector` windows appeared on the right. There is the `Is Active` checkbox. Check it and enjoy the sample motion!

## How to use in your application

### Spawn a tensegrity manipulator
- Place the `Assets/tm40` prefab. Its position will be the center of the base polygon.
- The procedural generation of tensegrity manipulator is written in `Assets/Project/Script/Assembly.cs`.
- The `Assembly.cs` has following parameters:
  - `Base Plate` - `Vertical Line`: 3D Objects constituting the structure.
  - `Num Layer`: The number of layers stucked vertically.
  - `Num Prism`: The number of struts in a layer.
  - `Radius Base`: Radius of the circle where the base is placed.
  - `Edge Loop`: Array holding the lengths of single edge of loop cables drawn in black.
  - `Is Paused Start`: If checked, physical simulation will be paused after placing all parts.
  - `Is Never Sleep`: If checked, the automatic sleep of PhysX is disabled so that the   changes will be reflected immediately.
- The `Assembly.cs` has the highest execution priority than other scripts in `Assets/Project/Script`.

![spawn](https://github.com/BSL-Kyutech/tmsim/assets/9003707/2b69008a-9e41-47a7-9b20-a0e24afc13f4)

### Make the tensegrity manipulator to be drivable
- The `Assets/tm40` prefab also has `Assets/Project/Script/Device.cs` as a component.
- The `Device.cs` mediates access to variables by numbering them in the appropriate order:
  - The indexes are numbered consecutively in counter-clockwise order on each layer, from the bottom to the top.
  - An actuator's index plus/minus `Num Prism` indicates the actuator on the same side of the upper/lower layer.
- The `Device.cs` has following parameters:
  - `Input`: Normalized input for actuators. If `Is Never Sleep` is checked, the change made in the inspector will be reflected immediately.
  - `Cylinder`: Current spring coefficients of SpringJoint components corresponding to actuators [`N/m`].
  - `Loop Position`: Current averaged positions of nodes constituting loop cables.
  - `Strut Position`: Current positions of struts.
  - `Strut Orientation`: Current orientation of struts.
  - `Max Delta`: Allowable spring constant change per fixed timestep of physical simulation. The fixed timestep is set to 0.001 [`sec`].

### Perform demonstrations
- Place the `Assets/ctrl` prefab. Its position will not have any meaning.
- This has `Assets/Project/Script/Demo.cs` as a component.
- The `Demo.cs` accesses to the `Input` of `Device.cs` and feeds dynamic input pattern for the demo motion.
- The `Demo.cs` has following parameters:
  - `Dev`: The instance of `Device.cs` that the target tensegrity manipulator object has.
  - `Is Active`: The dynamic input pattern and the `Input ` are allowed to be updated while this is checked.
  - `Omega` and `Theta`: Parameters for the input pattern generation. Read `Demo.cs` for more details.

### Let the controller talk ROS2
- The `Assets/ctrl` prefab also has `Assets/Project/Script/ROS2Interface.cs` as a component.
- The `ROS2Interface.cs` publishes/subscribes topics for variables of `Device.cs` using `ros2-for-unity`.
- The `ROS2Interface.cs` has following parameters:
  - `Dev`: The instance of `Device.cs` that the target tensegrity manipulator object has.
  - `Publish Rate`: Period of publishing [`sec`].
- The topics that the `ROS2Interface.cs` publishes/subscribes are following:
  - `$"/{dev.name}/loop_center{i}"`: (Publish) `Loop Position` of `Dev` stored in `geometry_msgs.msg.PointStamped` type message.
  - `$"/{dev.name}/strut{i*dev.numPrism+j}"`: (Publish) `Strut Position` and `Strut Orientation` of `Dev` stored in `geometry_msgs.msg.PoseStamped` type message.
  - `"/{dev.name}/input"`: (Subscribe) `Input` of `Dev` stored in `std_msgs.msg.Float32MultiArray` type message.
