using Microsoft.MixedReality.Toolkit.Examples.Demos;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.XR.ARFoundation;

using RosSpotJoint = RosMessageTypes.UnityRoboticsDemo.SpotJointMsg;

public class ROS2Subscriber : MonoBehaviour
{
    private void Awake()
    {

    }

    void Start()
    {
        //ROSConnection.GetOrCreateInstance().Subscribe<RosSpotJoint>("joints", RecieveMsg);
    }

    void Update()
    {
        // Recieve ROS2 messages
        ; // TODO
        // Body
        ; // TODO
        // Joints
        ; // TODO
    }

    /*
    void RecieveMsg(RosSpotJoint message)
    {
        SpotJoint.
        Debug.Log(message.name);
    }
    */
}
