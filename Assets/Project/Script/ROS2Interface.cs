using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class ROS2Interface : MonoBehaviour
{
    public Device dev;

    private ROS2UnityCore ros2Unity;
    private ROS2Node ros2Node;

    // IMU data publishers 
    private IPublisher<geometry_msgs.msg.Vector3>[] linearAccPub;
    private IPublisher<geometry_msgs.msg.Vector3>[] angularVelPub;

    // Command subscriber
    private ISubscription<std_msgs.msg.Float32MultiArray> inputSub;

    // Throttle
    private int fixedUpdateCounter = 0;
    private const int IMU_PUBLISH_DIVIDER = 4;  // 250 Hz

    // Coordinate conversions 
    public Vector3 ConvVecU2R(Vector3 unityVec) { return new Vector3(unityVec.z, -unityVec.x, unityVec.y); }
    public Vector3 ConvVecR2U(Vector3 rosVec) { return new Vector3(-rosVec.y, rosVec.z, rosVec.x); }
    public Quaternion ConvQuaU2R(Quaternion unityQua) { return new Quaternion(unityQua.z, -unityQua.x, unityQua.y, -unityQua.w); }
    public Quaternion ConvQuaR2U(Quaternion rosQua) { return new Quaternion(-rosQua.y, rosQua.z, rosQua.x, -rosQua.w); }

    void Start()
    {
        ros2Unity = new ROS2UnityCore();
        int numStruts = dev.numLayer * dev.numPrism;

        linearAccPub = new IPublisher<geometry_msgs.msg.Vector3>[numStruts];
        angularVelPub = new IPublisher<geometry_msgs.msg.Vector3>[numStruts];

        if (ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");

            // IMU publishers 
            for (int i = 0; i < numStruts; i++)
            {
                linearAccPub[i] = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>(
                    $"/{dev.name}/strut{i}/accel");
                angularVelPub[i] = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>(
                    $"/{dev.name}/strut{i}/gyro");
            }

            // Input subscription – full array length
            int totalInputs = dev.numLayer * dev.numPrism * 2;
            inputSub = ros2Node.CreateSubscription<std_msgs.msg.Float32MultiArray>(
                $"/{dev.name}/input",
                msg => {
                    for (int i = 0; i < totalInputs; i++)
                        dev.input[i] = (float)msg.Data[i];
                });

         
        }
    }

    void FixedUpdate()
    {
        if (!ros2Unity.Ok()) return;

        fixedUpdateCounter++;
        if (fixedUpdateCounter % IMU_PUBLISH_DIVIDER != 0) return;

        // Publish IMU data as separate Vector3 messages
	for (int i = 0; i < dev.numLayer * dev.numPrism; i++)
	{
	    // Get the transform of the current strut
	    Transform strutTransform = dev.GetStrut(i).transform;

	    // Compute specific force , world frame  to body frame
	    Vector3 worldAcc = dev.strutAcceleration[i];
	    Vector3 specificForceWorld = worldAcc - Physics.gravity;
	    Vector3 specificForceBody = strutTransform.InverseTransformDirection(specificForceWorld);

	    // Compute angular velocity ,world frame  to body frame
	    Vector3 angularVelWorld = dev.strutAngularVelocity[i];
	    Vector3 angularVelBody = strutTransform.InverseTransformDirection(angularVelWorld);

	    // Convert to ROS coordinate convention
	    Vector3 accRos = ConvVecU2R(specificForceBody);
	    Vector3 angRos = ConvVecU2R(angularVelBody);

	    // Publish
	    var accMsg = new geometry_msgs.msg.Vector3();
	    accMsg.X = accRos.x; accMsg.Y = accRos.y; accMsg.Z = accRos.z;
	    linearAccPub[i].Publish(accMsg);

	    var angMsg = new geometry_msgs.msg.Vector3();
	    angMsg.X = angRos.x; angMsg.Y = angRos.y; angMsg.Z = angRos.z;
	    angularVelPub[i].Publish(angMsg);
	}
    }

    void Update() { }
}
