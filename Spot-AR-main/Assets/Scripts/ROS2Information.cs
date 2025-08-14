using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;


public class ROS2Information : MonoBehaviour
{
    private void Start()
    {
        Debug.Log(ROSConnection.RosIPAddressPref.ToString());
        Debug.Log(ROSConnection.RosPortPref.ToString());
        //ROSConnection.GetOrCreateInstance();
        //ROSConnection.SetIPPref("0.0.0.0");
        //ROSConnection.SetPortPref(21150);
    }

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.L))
        {
            //UnityEngine.PlayerPrefs.SetString(ROSConnection.PlayerPrefsKey_ROS_IP, "192.168.200.100");
            //UnityEngine.PlayerPrefs.SetInt(ROSConnection.PlayerPrefsKey_ROS_TCP_PORT, 21150);
            //ROSConnection.GetOrCreateInstance();
            ROSConnection.GetOrCreateInstance().Connect("192.168.200.100", 21150);
            //ROSConnection.GetOrCreateInstance().
            //ROSConnection.GetOrCreateInstance();
        }
    }
}
