using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

using RosHello = RosMessageTypes.UnityRoboticsDemo.HelloTestMsg;

public class HelloTestSubscriber : MonoBehaviour
{

    public ROS2Manager ros2Manager;
    //public TextMeshProUGUI helloCounterText;

    private string helloTopic = "hello";

    private int helloCount = 0;

    public EventHandler<int> helloReceived;

    // Start is called before the first frame update
    void Start()
    {
        ros2Manager.rosConnectedEvent += AddSubscribers;
        ros2Manager.rosDisconnectedEvent += RemoveSubscribers;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void AddSubscribers(object sender, EventArgs e)
    {
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null)
        {
            Debug.Log("Subscribing to: " + helloTopic);
            ros.Subscribe<RosHello>(helloTopic, UpdateHelloCounter);
            ros2Manager.NotifyAboutSubscriberTopic(helloTopic, true);
        }
    }

    private void RemoveSubscribers(object sender, ROSConnection ros)
    {
        Debug.Log("Unsubscribing to: " + helloTopic);
        ros.Unsubscribe(helloTopic);
        ros2Manager.NotifyAboutSubscriberTopic(helloTopic, false);
    }

    private void UpdateHelloCounter(RosHello message)
    {
        helloCount = helloCount + 1;
        helloReceived.Invoke(this, helloCount);
        //helloCounterText.text = helloCount.ToString();
    }
}
