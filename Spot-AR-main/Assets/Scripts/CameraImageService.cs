using JetBrains.Annotations;
using RosMessageTypes.Sensor;
using RosMessageTypes.UnityRoboticsDemo;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Data.SqlTypes;
using System.IO;
using System.Linq;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using UnityEngine;
using UnityEngine.UI;
using RosCameraRequest = RosMessageTypes.UnityRoboticsDemo.GetSpotCameraRequest;
using RosCameraResponse = RosMessageTypes.UnityRoboticsDemo.GetSpotCameraResponse;

public class CameraImageService : MonoBehaviour
{

    private ROS2Manager ros2Manager;
    //public Image cameraFeedImage;
    public RawImage cameraFeedImage;
    public TextMeshProUGUI cameraNameText;

    private string topic = "get_spot_image";
    private string cameraName = "frontright_fisheye_image";
    private string pixelFormat = "PIXEL_FORMAT_RGB_U8";

    private bool serviceRegistered = false;

    private float requestRate = 1.0f / 2.0f; // ROS2 service request FPS
    private float timeElapsed = 0; // Used to space out messages more    

    private void Awake()
    {
        ros2Manager = GameObject.FindObjectOfType<ROS2Manager>();
    }

    void Start()
    {
        ros2Manager.rosConnectedEvent += SetCameraStreamActive;
        ros2Manager.rosDisconnectedEvent += SetCameraStreamDeactive;

        // ROS2 has already been connected, so start camera stream using the connection
        if(ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected)
        {
            SetCameraStreamActive(this, null);
        }
    }


    void Update()
    {
        timeElapsed += Time.deltaTime; // Update time ticker
        if (timeElapsed > requestRate)
        {
            if(serviceRegistered == true)
            {
                // Send request to update camera feed when it recieves response
                RosCameraRequest request = new RosCameraRequest(cameraName, pixelFormat, 50);
                ros2Manager.GetROSConnection().SendServiceMessage<RosCameraResponse>(topic, request, UpdateCameraFeed);
            }

            timeElapsed = 0; // Reset time ticker
        }
    }

    private void SetCameraStreamActive(object sender, EventArgs e)
    {
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null)
        {
            //ros2Manager.GetROSConnection().ImplementService<RosCameraRequest, RosCameraResponse>(topic, GetObjectPose);
            ros2Manager.GetROSConnection().RegisterRosService<RosCameraRequest, RosCameraResponse>(topic);
            serviceRegistered = true;
            ros2Manager.NotifyAboutSubscriberTopic(topic, true);
        }
    }

    private void SetCameraStreamDeactive(object sender, ROSConnection ros)
    {
        ; // TODO
        serviceRegistered = false;
        ros2Manager.NotifyAboutSubscriberTopic(topic, false);
    }

    private void UpdateCameraFeed(RosCameraResponse response)
    {
        //Debug.Log(response.image.ToString());
        //Debug.Log(response.image.data.Length);

        // Convert the response image to a texture
        Texture2D tex = new Texture2D((int)response.image.width, (int)response.image.height);
        for(int i = 0; i < response.image.width * response.image.height; i++)
        {
            int b = i * 3;

            byte blue = response.image.data[b];
            byte green = response.image.data[b + 1];
            byte red = response.image.data[b + 2];
            byte alpha = 255;

            int y = (int)(i / (int)response.image.width);
            int x = (int)(i % (int)response.image.width);

            tex.SetPixel(x, y, new Color32(red, green, blue, alpha));
        }

        /*
        Texture2D tex = new Texture2D(2, 2);
        ImageConversion.LoadImage(tex, response.image.data);
        */

        // Apply texture to a Unity canvas
        tex.Apply();
        cameraFeedImage.texture = tex;
        cameraNameText.text = cameraName;

        // Adjust canvas to match image dimensions
        //cameraFeedImage.rectTransform.sizeDelta = new Vector2(1.0f, (float)response.image.width / (float)response.image.height);
        cameraFeedImage.rectTransform.sizeDelta = new Vector2((float)response.image.width / (float)response.image.height, 1.0f);
    }

    public void SetCamera(string cameraName)
    {
        this.cameraName = cameraName;
    }

    public void Close()
    {
        GameObject.Destroy(transform.parent.gameObject);
    }
}
