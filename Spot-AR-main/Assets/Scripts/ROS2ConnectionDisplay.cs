using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ROS2ConnectionDisplay : MonoBehaviour
{
    [Header("Main Window - Primary Display")]
    public TextMeshProUGUI textIP;
    public TextMeshProUGUI textPort;
    public TextMeshProUGUI textParticipantID;
    public Image imageConnectionDisplay;
    public Image alertDisplay;
    [Header("Main Window - Secondary")]
    public TextMeshProUGUI textPublishingTopics;
    public TextMeshProUGUI textSubscribingTopics;
    public TextMeshProUGUI textHello;
    public Image recordingDisplay;
    public TextMeshProUGUI recordingText;
    //public TextMeshProUGUI textControlType;
    //public TextMeshProUGUI textStopped;
    [Header("Top Display Window")]
    public Image spotJointsDisplay;
    public Image spotTransformDisplay;
    public Image spotAprilTagDisplay;
    public Image hl2QRDisplay;

    [Header("Scene Links")]
    public ROS2Manager ros2Manager;
    public VelocityManager velocityManager;
    public HelloTestSubscriber helloTestSubscriber;
    public SpotManager spotManager;
    public AnchorManager anchorManager;
    public TransformRecordingManager transformRecordingManager;
    public ParticipantLogger participantLogger;

    [Header("Connection/Display Parameters")]
    public Color connectedColor = Color.green;
    public Color disconnectedColor = Color.red;
    public float connectionTimeoutDuration = 1.0f;

    private DateTime lastSpotJointsReceivedTime = DateTime.MaxValue;
    private DateTime lastSpotTransformReceivedTime = DateTime.MaxValue;
    private DateTime lastSpotAprilTagReceivedTime = DateTime.MaxValue;
    //private float lastSpotTransformTimeout = 0f;
    //private float lastSpotAprilTagTimeout = 0f;

    //private IEnumerator alertCoroutine = null;

    private void Awake()
    {
        textPublishingTopics.text = ""; // Reset topic display
        textSubscribingTopics.text = ""; // Reset topic display

        //velocityManager.controlTypeChanged += ControlTypeChanged;
        //velocityManager.pointCommandIssueFailed += MakeMenuFlashRed;
        helloTestSubscriber.helloReceived += UpdateHelloCountDisplay;
        spotManager.validSpotJointsReceived += UpdateIndicatorSpotJoints;
        spotManager.validSpotTransformReceived += UpdateIndicatorSpotTransform;
        spotManager.validSpotAprilTagReceived += UpdateIndicatorSpotAprilTag;
        anchorManager.validQRScanned += UpdateIndicatorHL2Tag;
        transformRecordingManager.recordingStatusChanged += UpdateRecordingIndicator;

        SetTimeoutIndicatorDisonnected(spotJointsDisplay);
        SetTimeoutIndicatorDisonnected(spotTransformDisplay);
        SetTimeoutIndicatorDisonnected(spotAprilTagDisplay);
        SetTimeoutIndicatorDisonnected(hl2QRDisplay);
    }

    private void Start()
    {

    }

    private void Update()
    {
        UpdateConnectionDisplay();
        UpdateTopicDisplay();

        // DisplayTimeouts
        /*
        lastSpotTransformTimeout += Time.deltaTime;
        lastSpotAprilTagTimeout += Time.deltaTime;
        if(lastSpotTransformTimeout > connectionTimeoutDuration)
        {
            SetTimeoutIndicatorDisonnected(spotTransformDisplay);
        }
        if(lastSpotAprilTagTimeout > connectionTimeoutDuration)
        {
            SetTimeoutIndicatorDisonnected(spotAprilTagDisplay);
        }
        */
        if ((DateTime.Now - lastSpotJointsReceivedTime).TotalSeconds > connectionTimeoutDuration)
        {
            SetTimeoutIndicatorDisonnected(spotJointsDisplay);
        }
        if ((DateTime.Now - lastSpotTransformReceivedTime).TotalSeconds > connectionTimeoutDuration)
        {
            SetTimeoutIndicatorDisonnected(spotTransformDisplay);
        }
        if ((DateTime.Now - lastSpotAprilTagReceivedTime).TotalSeconds > connectionTimeoutDuration)
        {
            SetTimeoutIndicatorDisonnected(spotAprilTagDisplay);
        }
    }

    public void UpdateConnectionDisplay()
    {
        // Connection text
        textIP.text = ros2Manager.GetIP().ToString();
        textPort.text = ros2Manager.GetPort().ToString();
        textParticipantID.text = participantLogger.GetParticipantID();
        // Active connection icon
        ROS2Manager.ROS2ConnectionStatus status = ros2Manager.GetStatus();
        if(status == ROS2Manager.ROS2ConnectionStatus.Connected)
        {
            imageConnectionDisplay.color = connectedColor;
        }
        else if(status == ROS2Manager.ROS2ConnectionStatus.Disconnected)
        {
            imageConnectionDisplay.color = disconnectedColor;
        }
        else if(status == ROS2Manager.ROS2ConnectionStatus.Inactive)
        {
            imageConnectionDisplay.color = Color.white;
        }
        else
        {
            imageConnectionDisplay.color = Color.white;
        }
    }

    public void UpdateTopicDisplay()
    {
        textPublishingTopics.text = ""; // Reset topic display
        textSubscribingTopics.text = ""; // Reset topic display
        foreach (string topic in ros2Manager.GetActivePublishingTopics())
        {
            textPublishingTopics.text += topic + "\n";
        }
        foreach (string topic in ros2Manager.GetActiveSubscriberTopics())
        {
            textSubscribingTopics.text += topic + "\n";
        }
    }

    /*
    private void ControlTypeChanged(object sender, string e)
    {
        if (e == "False")
            textControlType.text = "Velocity";
        else
            textControlType.text = "Point";
    }
    */

    private void UpdateHelloCountDisplay(object sender, int e)
    {
        textHello.text = e.ToString();
    }

    /*
    private void MakeMenuFlashRed(object sender, EventArgs e)
    {
        CancelAlert();
        if (this.isActiveAndEnabled)
        {
            alertDisplay.color = new Color(1.0f, 0f, 0f, 1.0f);
            alertCoroutine = FadeAlertToTransparent();
            StartCoroutine(alertCoroutine);
        }
    }

    public void CancelAlert()
    {
        alertDisplay.color = new Color(1.0f, 0f, 0f, 0.0f);
        if (alertCoroutine != null)
        {
            StopCoroutine(alertCoroutine);
            alertCoroutine = null;
        }
    }

    private IEnumerator FadeAlertToTransparent()
    {
        int max = 20;
        for(int i = max; i > 0; i--) 
        {
            alertDisplay.color = new Color(1.0f, 0f, 0f, i / (float)max);
            yield return new WaitForSeconds(0.05f);
        }
        alertDisplay.color = new Color(1.0f, 0f, 0f, 0f);
    }
    */

    private void UpdateIndicatorSpotJoints(object sender, EventArgs e)
    {
        SetTimeoutIndicatorConnected(spotJointsDisplay);
        lastSpotJointsReceivedTime = DateTime.Now;
    }

    private void UpdateIndicatorSpotTransform(object sender, EventArgs e)
    {
        SetTimeoutIndicatorConnected(spotTransformDisplay);
        lastSpotTransformReceivedTime = DateTime.Now;
        //lastSpotTransformTimeout = 0f;
    }

    private void UpdateIndicatorSpotAprilTag(object sender, EventArgs e)
    {
        SetTimeoutIndicatorConnected(spotAprilTagDisplay);
        lastSpotAprilTagReceivedTime = DateTime.Now;
        //lastSpotAprilTagTimeout = 0f;
    }

    private void UpdateIndicatorHL2Tag(object sender, EventArgs e)
    {
        SetTimeoutIndicatorConnected(hl2QRDisplay);
    }

    private void SetTimeoutIndicatorConnected(Image timeoutDisplay)
    {
        timeoutDisplay.color = connectedColor;
    }

    private void SetTimeoutIndicatorDisonnected(Image timeoutDisplay)
    {
        timeoutDisplay.color = disconnectedColor;
    }

    private void UpdateRecordingIndicator(object sender, bool e)
    {
        recordingDisplay.gameObject.SetActive(e);
        recordingText.gameObject.SetActive(e);
    }
}
