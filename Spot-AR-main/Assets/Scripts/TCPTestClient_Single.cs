/*
 * Adapted from:
 * https://gist.github.com/danielbierwirth/0636650b005834204cb19ef5ae6ccedb
 * By John (6/27/2023)
 */

// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using TMPro;
using UnityEngine;

public class TCPTestClient_Single : MonoBehaviour
{
    public string hostAddress = "127.0.0.1";
    public int portID = 21001;
    public int bufferSize = 1024;

    public TextMeshPro displayText;

    #region private members 	
    private TcpClient socketConnection;
    private Thread clientReceiveThread;
    #endregion

    // Use this for initialization 	
    private void Start()
    {
        //ConnectToTcpServer();
    }

    // Update is called once per frame
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            LaunchPingThread();
        }
    }

    public void LaunchPingThread()
    {
        clientReceiveThread = new Thread(new ThreadStart(PingForData));
        clientReceiveThread.IsBackground = true;
        clientReceiveThread.Start();
    }

    /*
    /// <summary> 	
    /// Setup socket connection. 	
    /// </summary> 	
    private bool ConnectToTcpServer()
    {
        try
        {
            clientReceiveThread = new Thread(new ThreadStart(ListenForData));
            clientReceiveThread.IsBackground = true;
            clientReceiveThread.Start();
            return true; // success
        }
        catch (Exception e)
        {
            Debug.Log("On client connect exception " + e);
            return false; // success
        }
    }
    */

    /// <summary> 	
    /// Runs in background clientReceiveThread; Listens for incomming data. 	
    /// </summary>     
    private void PingForData()
    {
        try
        {
            socketConnection = new TcpClient(hostAddress, portID);
            Byte[] bytes = new Byte[bufferSize];
            while (true)
            {
                // Get a stream object for reading 				
                using (NetworkStream stream = socketConnection.GetStream())
                {
                    int length;
                    // Read incomming stream into byte arrary. 					
                    while ((length = stream.Read(bytes, 0, bytes.Length)) != 0)
                    {
                        var incommingData = new byte[length];
                        Array.Copy(bytes, 0, incommingData, 0, length);
                        // Convert byte array to string message. 						
                        string serverMessage = Encoding.ASCII.GetString(incommingData);
                        Debug.Log("server message received as: " + serverMessage);
                        //displayText.text = serverMessage;
                    }
                }
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
        }
    }

    /*
    /// <summary> 	
    /// Send message to server using socket connection. 	
    /// </summary> 	
    private void SendMessage()
    {
        if (socketConnection == null)
        {
            return;
        }
        try
        {
            // Get a stream object for writing. 			
            NetworkStream stream = socketConnection.GetStream();
            if (stream.CanWrite)
            {
                string clientMessage = "This is a message from one of your clients.";
                // Convert string message to byte array.                 
                byte[] clientMessageAsByteArray = Encoding.ASCII.GetBytes(clientMessage);
                // Write byte array to socketConnection stream.                 
                stream.Write(clientMessageAsByteArray, 0, clientMessageAsByteArray.Length);
                Debug.Log("Client sent his message - should be received by server");
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
        }
    }
    */
}