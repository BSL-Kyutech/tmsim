using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;

public class ROS2Interface : MonoBehaviour
{
    public Device dev;

    //
    private ROS2UnityCore ros2Unity;
    //ノードを作成するための最初の処理
    private ROS2Node ros2Node;
    //配信する側のグローバル変数の作成
    private IPublisher<geometry_msgs.msg.PointStamped>[] loopPosPub;
    private IPublisher<geometry_msgs.msg.PoseStamped>[] strutPosPub;
    //受信する側のグローバル変数の作成
    private ISubscription<std_msgs.msg.Float32MultiArray> inputSub;

    //InvokeRepaetingのリピートのためのリピートする量
    public float publishRate = 0.01f; //sec
    
    
    public Vector3 ConvVecU2R( Vector3 unityVec)
    {
        return new Vector3(unityVec.z, -unityVec.x, unityVec.y);
    }

    public Vector3 ConvVecR2U( Vector3 rosVec)
    {
        return new Vector3(-rosVec.y, rosVec.z, rosVec.x);
    }

    public Quaternion ConvQuaU2R( Quaternion unityQua)
    {
        return new Quaternion(unityQua.z, -unityQua.x, unityQua.y, -unityQua.w);
    }

    public Quaternion ConvQuaR2U( Quaternion rosQua)
    {
        return new Quaternion(-rosQua.y, rosQua.z, rosQua.x, -rosQua.w);
    }
    

    // Start is called before the first frame update
    void Start()
    {
        //ROS2とUnityクラスの呼び出し
        ros2Unity = new ROS2UnityCore();
        //紐に関する外への通信(配信を行う)Pubの生成(グローバル変数への割り当て)
        loopPosPub = new IPublisher<geometry_msgs.msg.PointStamped>[dev.numLayer+1];
        //Strutに関する外へのROS通信(配信を行う)Pubの生成(グローバル変数への割り当て)
        strutPosPub = new IPublisher<geometry_msgs.msg.PoseStamped>[dev.numLayer*dev.numPrism];

        if (ros2Unity.Ok()) //Nodeを立てるにはROS2が初期化されている必要があるため，正しく初期化されているか確認
        {

            //ROSのNodeの生成 (ROSにかかわる処理を行うプログラムの実行する処理を担うオブジェクト?)
            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            for (int i = 0; i < dev.numLayer+1; i++) 
            {   //データを受け取り，Nodeの作成(おそらく，配列内にloop_center番号の割り当て?)
                loopPosPub[i] = ros2Node.CreatePublisher<geometry_msgs.msg.PointStamped>($"/{dev.name}/loop_center{i}"); 
            }
            for (int i = 0; i < dev.numLayer; i++) 
            {
                for (int j = 0; j < dev.numPrism; j++) 
                {   
                    //strutのポジションの配列のノードの作成
                    strutPosPub[i*dev.numPrism+j] = ros2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>($"/{dev.name}/strut{i*dev.numPrism+j}"); 
                }
            }

            // register a subscription callback
            inputSub = ros2Node.CreateSubscription<std_msgs.msg.Float32MultiArray>($"/{dev.name}/input", 
            msg => {
                for (int i = 0;i < dev.numLayer*dev.numPrism; i++) 
                {
                    dev.input[i] = (float)msg.Data[i];
                }
            });
            InvokeRepeating(nameof(PublisherCallback), 1.0f, publishRate); // Publish every second
        }
    }

    void PublisherCallback()
    {

        for (int i = 0; i < dev.numLayer+1; i++) {
            geometry_msgs.msg.PointStamped msg = new geometry_msgs.msg.PointStamped();
            var pos = ConvVecU2R(dev.loopPosition[i]);
            msg.Point.X = pos.x;
            msg.Point.Y = pos.y;
            msg.Point.Z = pos.z;
            loopPosPub[i].Publish(msg);
        }
        
        for (int i = 0; i < dev.numLayer; i++) 
        {
            for (int j = 0; j < dev.numPrism; j++) 
            {
                geometry_msgs.msg.PoseStamped msg = new geometry_msgs.msg.PoseStamped();
                var point = ConvVecU2R(dev.strutPosition[i]);
                var orientation = ConvQuaU2R(dev.strutOrientation[i]);
                msg.Pose.Position.X = point.x;
                msg.Pose.Position.Y = point.y;
                msg.Pose.Position.Z = point.z;
                msg.Pose.Orientation.X = orientation.x;
                msg.Pose.Orientation.Y = orientation.y;
                msg.Pose.Orientation.Z = orientation.z;
                msg.Pose.Orientation.W = orientation.w;
                strutPosPub[i*dev.numPrism+j].Publish(msg);
            }
        }

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
