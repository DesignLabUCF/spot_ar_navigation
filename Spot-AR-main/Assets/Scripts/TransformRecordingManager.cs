using RosMessageTypes.UnityRoboticsDemo;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

using RosSpotRecordingHelperRequest = RosMessageTypes.UnityRoboticsDemo.TransformRecordingServiceInfoRequest;
using RosSpotRecordingHelperResponse = RosMessageTypes.UnityRoboticsDemo.TransformRecordingServiceInfoResponse;


public class TransformRecordingManager : MonoBehaviour
{
    public ROS2Manager ros2Manager;

    private string topic = "recording_helper";

    private bool isRecording = false;

    public EventHandler<bool> recordingStatusChanged;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ReceiveRecordingButtonInput()
    {
        // Check if recording should happen
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null) // ROS has connected
        {
            // Register the topic
            if (ros2Manager.ShouldRegisterTopic(topic))
            {
                ros.RegisterRosService<RosSpotRecordingHelperRequest, RosSpotRecordingHelperResponse>(topic);
                ros2Manager.NotifyAboutPublishingTopic(topic, true);
            }

            RosSpotRecordingHelperRequest request = new RosSpotRecordingHelperRequest(true);

            ros.SendServiceMessage<RosSpotRecordingHelperResponse>(topic, request, ReceiveStatusResponse);
        }

        ; // TODO
        // TODO verify recording should take place with ROS. Send service, check response, send another service?

        /*
        if(isRecording == false)
        {
            StartRecording();
        }
        else
        {
            StopRecording();
        }
        */
    }

    private void ReceiveStatusResponse(RosSpotRecordingHelperResponse response)
    {
        Debug.Log("Initial request response - is_recording: " + response.is_recording);
        Debug.Log("Initial request response - succesfully_started_recording: " + response.succesfully_started_recording);
        Debug.Log("Initial request response - succesfully_started_recording: " + response.able_to_record);
    }

    private void StartRecording()
    {
        isRecording = true;
        recordingStatusChanged.Invoke(this, isRecording);
    }

    private void StopRecording()
    {
        isRecording = false;
        recordingStatusChanged.Invoke(this, isRecording);
    }
}
