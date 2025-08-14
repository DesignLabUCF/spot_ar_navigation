using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq.Expressions;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class FiducialFollowManager : MonoBehaviour
{
    public string hostAddress = "127.0.0.1";
    public int portID = 21006;
    public int bufferSize = 1024;

    private bool isActive = false;
    private string sendMsg = "";

    private ROS2Manager ros2Manager;

    private TcpClient socketConnection;
    private Thread clientReceiveThread;


    private void Start()
    {
        ros2Manager = FindObjectOfType<ROS2Manager>();
    }

    private void Update()
    {
        // Pull from ROS2 manager (set by ROS2 hand menu)
        hostAddress = ros2Manager.GetIP();
        portID = ros2Manager.GetPort();
    }

    public void LaunchPingThread()
    {
        clientReceiveThread = new Thread(new ThreadStart(PingContainer));
        clientReceiveThread.IsBackground = true;
        clientReceiveThread.Start();
    }

    private void PingContainer()
    {
        isActive = true;
        try
        {
            socketConnection = new TcpClient(hostAddress, portID);
            using (NetworkStream stream = socketConnection.GetStream())
            {
                Byte[] data = new Byte[bufferSize];
                String responseData = String.Empty;
                Int32 byteRec = 0;

                // Receive
                byteRec = stream.Read(data, 0, data.Length);
                responseData = System.Text.Encoding.ASCII.GetString(data, 0, byteRec);
                Debug.Log("Received: " + responseData);

                // Send
                byte[] msg = Encoding.ASCII.GetBytes(sendMsg);
                stream.Write(msg, 0, msg.Length);
                Debug.Log("Sending: " + msg.ToString());

                // Receive
                byteRec = stream.Read(data, 0, data.Length);
                responseData = System.Text.Encoding.ASCII.GetString(data, 0, byteRec);
                Debug.Log("Received: " + responseData);

                // Receive
                //byteRec = stream.Read(data, 0, data.Length);
                //responseData = System.Text.Encoding.ASCII.GetString(data, 0, byteRec);
                //Debug.Log("Received: " + responseData);
            }
            /*
            Byte[] bytes = new Byte[bufferSize];
            while (true)
            {
                // Get a stream object for reading 				
                using (NetworkStream stream = socketConnection.GetStream())
                {
                    data = new Byte[256];
                    stream.Read(data, 0, data.Length);

                    int length;
                    // Read incomming stream into byte arrary. 					
                    while ((length = stream.Read(bytes, 0, bytes.Length)) != 0)
                    {
                        var incomingData = new byte[length];
                        Array.Copy(bytes, 0, incomingData, 0, length);
                        // Convert byte array to string message. 						
                        string serverMessage = Encoding.ASCII.GetString(incomingData);
                        Debug.Log("server message received as: " + serverMessage);
                        //displayText.text = serverMessage;
                    }
                }
            }
            */

        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
        }
        isActive = false;
    }

    public void LaunchFF()
    {
        if(isActive == false)
        {
            sendMsg = "True";
            LaunchPingThread();
        }
    }

    public void StopFF()
    {
        if (isActive == false)
        {
            sendMsg = "False";
            LaunchPingThread();
        }
    }
}
