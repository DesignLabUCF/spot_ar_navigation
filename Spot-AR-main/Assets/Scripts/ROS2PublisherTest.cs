using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

/// <summary>
///
/// </summary>
public class ROS2PublisherTest : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    private bool publishing = false;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);

        publishing = true;
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            cube.transform.rotation = Random.rotation;

            PosRotMsg cubePos = new PosRotMsg(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, cubePos);

            timeElapsed = 0;
        }
    }

    public bool IsPublishing()
    {
        return publishing;
    }
}

/*
using RosMessageTypes.UnityRoboticsDemo;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ROS2PublisherTest : MonoBehaviour
{
    private ROS2Manager ros2Manager;
    private bool publishing = false;

    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // start the ROS connection
        //ros = ROSConnection.GetOrCreateInstance();
        //ros.RegisterPublisher<PosRotMsg>(topicName);

        ros2Manager = FindObjectOfType<ROS2Manager>();
        StartPublishing();
    }

    private void Update()
    {
        if((publishing == true) && (ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected))
        {
            timeElapsed += Time.deltaTime;

            if (timeElapsed > publishMessageFrequency)
            {
                PosRotMsg cubePos = new PosRotMsg(
                    cube.transform.position.x,
                    cube.transform.position.y,
                    cube.transform.position.z,
                    cube.transform.rotation.x,
                    cube.transform.rotation.y,
                    cube.transform.rotation.z,
                    cube.transform.rotation.w
                );

                // Finally send the message to server_endpoint.py running in ROS
                ros.Publish(topicName, cubePos);

                timeElapsed = 0;
            }
        }
    }

    public void StartPublishing()
    {
        if(publishing == false)
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<PosRotMsg>(topicName);
            publishing = true;
        }
    }

    public bool IsPublishing()
    {
        return publishing;
    }
}
*/