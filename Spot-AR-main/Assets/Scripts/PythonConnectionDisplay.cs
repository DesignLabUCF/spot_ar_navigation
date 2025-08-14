using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class PythonConnectionDisplay : MonoBehaviour
{
    public TextMeshProUGUI textIP;
    public TextMeshProUGUI textPort;
    public Image imageConnectionDisplay;

    private ROS2Manager ros2Manager;
    private FiducialFollowManager fiducialFollowManager;

    private void Start()
    {
        ros2Manager = FindObjectOfType<ROS2Manager>();

        UpdateDisplay();
    }

    private void Update()
    {


        UpdateDisplay();
    }

    private void UpdateDisplay()
    {
        // Connection text
        textIP.text = ros2Manager.GetIP().ToString();
        textPort.text = ros2Manager.GetPort().ToString();
        // Active connection icon
        // TODO - Maybe
    }
}
