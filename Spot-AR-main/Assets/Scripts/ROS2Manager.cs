using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine.Events;
using System;
using System.Runtime.CompilerServices;

public class ROS2Manager : MonoBehaviour
{
    private ROSConnection ros = null;
    [SerializeField]
    private string IP = "192.168.200.178";
    [SerializeField]
    private int port = 21150;

    /*
    public TouchScreenKeyboard ipKeyboard;
    public static string ipKeyboardText = "";
    public TouchScreenKeyboard portKeyboard;
    public static string portKeyboardText = "";
    */

    private ROS2ConnectionStatus currentConnectionStatus = ROS2ConnectionStatus.Disconnected;
    private List<string> publishingTopics = new List<string>();
    private List<string> subscribingTopics = new List<string>();
    public EventHandler rosConnectedEvent;
    public EventHandler<ROSConnection> rosDisconnectedEvent;

    public enum ROS2ConnectionStatus
    {
        Connected,
        Disconnected,
        Inactive
    }

    private void Awake()
    {

    }

    private void Start()
    {

    }

    private void Update()
    {
        // TODO check if disconnected unexpectedly, then send out disconnect event
        ROS2ConnectionStatus status = GetStatus();
        if(status == ROS2ConnectionStatus.Disconnected && currentConnectionStatus == ROS2ConnectionStatus.Connected)
        {
            ConnectionInterruptionDetected();
        }
        currentConnectionStatus = status;// Update current status

        /*
        if (ipKeyboard != null)
        {
            IP = ipKeyboard.text.ToString();
        }
        if(portKeyboard != null)
        {
            port = int.Parse(portKeyboard.text);
        }
        */

#if UNITY_EDITOR
        if (Input.GetKeyDown(KeyCode.C))
        {
            Connect();
        }
        if (Input.GetKeyDown(KeyCode.D))
        {
            Disconnect();
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            Debug.Log(GetStatus());
        }
        if (Input.GetKeyDown(KeyCode.I))
        {
            //IP = "192.168.200.100";
            IP = "127.0.0.1";
        }
        if (Input.GetKeyDown(KeyCode.P))
        {
            port = 20000;
        }
# endif
    }

    public void SetIP(string IP)
    {
        this.IP = IP;
    }

    public void SetPort(int port)
    {
        this.port = port;
    }

    public void Connect()
    {
        if(GetStatus() == ROS2ConnectionStatus.Connected)
        {
            Debug.Log("ROS2 already connected. No need to connect");
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect(IP, port);

        Invoke("SendConnectedEventIfSuccessful", 0.5f); // Invoked after delay to ensure the process has time to be good/bad
    }

    public void Disconnect()
    {
        if (ros != null)
        {
            rosDisconnectedEvent.Invoke(this, ros);
            ros.Disconnect();
            ros = null;
        }
    }

    public void ConnectionInterruptionDetected()
    {
        Debug.Log("Disconnection occured.");
        Disconnect();
    }

    private void SendConnectedEventIfSuccessful()
    {
        if (GetStatus() == ROS2ConnectionStatus.Connected)
            rosConnectedEvent.Invoke(this, null);
    }

    public ROSConnection GetROSConnection()
    {
        if(GetStatus() == ROS2ConnectionStatus.Connected)
            return ros;
        else
            return null;
    }

    public bool ShouldRegisterTopic(string topic)
    {
        if (ros == null)
            return false;
        return (ros.GetTopic(topic) == null);
    }

    public void NotifyAboutPublishingTopic(string topic, bool nowPublishing)
    {
        if (nowPublishing)
        {
            publishingTopics.Add(topic);
        }
        else
        {
            publishingTopics.Remove(topic);
        }
    }

    public void NotifyAboutSubscriberTopic(string topic, bool nowSubscribing)
    {
        if (nowSubscribing)
        {
            subscribingTopics.Add(topic);
        }
        else
        {
            subscribingTopics.Remove(topic);
        }
    }

    public List<string> GetActivePublishingTopics()
    {
        return publishingTopics;
    }

    public List<string> GetActiveSubscriberTopics()
    {
        return subscribingTopics;
    }

    public ROS2ConnectionStatus GetStatus()
    {
        if (ros != null)
        {
            if (ros.HasConnectionThread)
            {
                if (ros.HasConnectionError)
                {
                    return ROS2ConnectionStatus.Disconnected;
                }
                else
                {
                    return ROS2ConnectionStatus.Connected;
                }
            }
            else
            {
                return ROS2ConnectionStatus.Inactive;
            }
        }
        else
        {
            return ROS2ConnectionStatus.Inactive;
        }
    }

    public string GetIP()
    {
        return IP;
    }

    public int GetPort()
    {
        return port;
    }
}
