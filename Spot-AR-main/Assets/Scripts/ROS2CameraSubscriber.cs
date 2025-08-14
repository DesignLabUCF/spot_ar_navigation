using System;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;
using RosSpotCameraImage = RosMessageTypes.UnityRoboticsDemo.SpotImageMsg;

public class ROS2CameraSubscriber : MonoBehaviour
{
    private ROS2Manager ros2Manager;
    public RawImage cameraFeedImage;
    public TextMeshProUGUI cameraNameText;

    private const string TOPIC_PREFIX = "spot_image_";
    private string cameraName = "";
    private string topic = "";
    private bool serviceRegistered = false;

    // Camera data
    byte[] imageData = null;
    int imageHeight = 0;
    int imageWidth = 0;
    string imageFormat = "";

    private Texture2D tex = null;

    private float timeoutTimeElapsed = 0f;
    private float timeoutLimit = 3.0f;


    private void Awake()
    {
        ros2Manager = GameObject.FindObjectOfType<ROS2Manager>();

        /*
        // Setup camera toggle
        Transform switchCollection = transform.Find("SwitchCollection");
        for (int i = 0; i < switchCollection.childCount; i++)
        {
            cameraViewToggles.Add(switchCollection.GetChild(i).GetComponent<Interactable>());
        }
        */

        tex = new Texture2D(1, 1);
    }

    private void Start()
    {
        ros2Manager.rosConnectedEvent += AddSubscribers;
        ros2Manager.rosDisconnectedEvent += RemoveSubscribers;

        /*
        // ROS2 has already been connected, so start camera stream using the connection
        if (ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected)
        {
            AddSubscribers(this, null);
        }
        */
    }

    private void Update()
    {
        timeoutTimeElapsed = timeoutTimeElapsed + Time.deltaTime;

        if (ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected && imageData != null && !TimeoutReached())
        {
            switch(imageFormat)
            {
                case "PIXEL_FORMAT_RGB_U8":
                    UpdateImage(TextureFormat.RGB24);
                    break;
                default:
                    Debug.LogError("Additional pixel formats for CameraView not yet implemented. Only 'PIXEL_FORMAT_RGB_U8' is available so far...");
                    break;
            }
        }
        else if(ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected && imageData != null && TimeoutReached()) // Seems to be some sort of camera not subscribing correctly issue when cameras. This leads to timeouts even though ROS2 is still connected. This is a quick fix and unsubscribes/resubscribes
        {
            SetCamera(cameraName);
        }
        else
        {
            BlankCameraFeed();
        }
        
    }

    private bool TimeoutReached()
    {
        return timeoutTimeElapsed > timeoutLimit;
    }

    private void SetImageData(RosSpotCameraImage msg)
    {
        // Image data
        imageData = msg.image.data;
        imageHeight = (int)msg.image.height;
        imageWidth = (int)msg.image.width;
        imageFormat = msg.pixel_format;
        // Time from last message checker
        timeoutTimeElapsed = 0f;
    }

    private void UpdateImage(TextureFormat imageFormat)
    {
        // Send previously created texture to garbage collector
        if (tex != null)
        {
            Destroy(tex);
            tex = null;
        }

        // Update texture
        /*
        tex = new Texture2D(imageWidth, imageHeight);
        for (int i = 0; i < imageWidth * imageHeight; i++)
        {
            int b = i * 3;

            byte blue = imageData[b];
            byte green = imageData[b + 1];
            byte red = imageData[b + 2];
            byte alpha = 255;

            int y = (int)(i / imageWidth);
            int x = (int)(i % imageWidth);

            tex.SetPixel(x, y, new Color32(red, green, blue, alpha));
        }
        */
        tex = new Texture2D(imageWidth, imageHeight, imageFormat, false);
        tex.LoadRawTextureData(imageData);

        // Apply texture to a Unity canvas
        tex.Apply();
        cameraFeedImage.texture = tex;
        cameraNameText.text = cameraName;

        // Adjust canvas display image appropriately
        cameraFeedImage.rectTransform.sizeDelta = new Vector2((float)imageWidth / (float)imageHeight, 1.0f);
        if(cameraName.Contains("frontright") || cameraName.Contains("frontleft"))
        {
            cameraFeedImage.rectTransform.localRotation = UnityEngine.Quaternion.Euler(0, 0, 270.0f);
            cameraFeedImage.rectTransform.localScale = new Vector3(1.0f, -1.0f, 1.0f);
        }
        else if(cameraName.Contains("left"))
        {
            cameraFeedImage.rectTransform.localScale = new Vector3(1.0f, -1.0f, 1.0f);
        }
        else if (cameraName.Contains("right"))
        {
            ;
        }
        else if(cameraName.Contains("back"))
        {
            cameraFeedImage.rectTransform.localScale = new Vector3(1.0f, -1.0f, 1.0f);
        }
    }


    private void UpdateCameraFeed(RosSpotCameraImage msg)
    {
        //Debug.Log(response.image.ToString());
        //Debug.Log(response.image.data.Length);

        // Convert the response image to a texture
        Texture2D tex = new Texture2D((int)msg.image.width, (int)msg.image.height);
        for (int i = 0; i < msg.image.width * msg.image.height; i++)
        {
            int b = i * 3;

            byte blue = msg.image.data[b];
            byte green = msg.image.data[b + 1];
            byte red = msg.image.data[b + 2];
            byte alpha = 255;

            int y = (int)(i / (int)msg.image.width);
            int x = (int)(i % (int)msg.image.width);

            tex.SetPixel(x, y, new Color32(red, green, blue, alpha));
        }

        /*
        Texture2D tex = new Texture2D(2, 2);
        ImageConversion.LoadImage(tex, response.image.data);
        */

        /*
        Texture2D tex = new Texture2D(1, 1);
        tex.SetPixel(0, 0, Color.yellow);
        */

        // Apply texture to a Unity canvas
        tex.Apply();
        cameraFeedImage.texture = tex;
        cameraNameText.text = cameraName;

        // Adjust canvas to match image dimensions
        //cameraFeedImage.rectTransform.sizeDelta = new Vector2(1.0f, (float)response.image.width / (float)response.image.height);
        cameraFeedImage.rectTransform.sizeDelta = new Vector2((float)msg.image.width / (float)msg.image.height, 1.0f);
    }

    /*
    private async void UpdateCameraFeedAsync(RosSpotCameraImage msg) // https://learn.microsoft.com/en-us/dotnet/csharp/language-reference/operators/await
    {
        // Convert the response image to a texture
        Texture2D tex = new Texture2D(2, 2); // is overwritten anyway

        Debug.Log("Test1");
        tex = await UpdateTexturePixels((int)msg.image.width, (int)msg.image.height, msg.image.data);
        Debug.Log("Test2");

        // Apply texture to a Unity canvas
        tex.Apply();
        cameraFeedImage.texture = tex;
        cameraNameText.text = cameraName;

        // Adjust canvas to match image dimensions
        cameraFeedImage.rectTransform.sizeDelta = new Vector2((float)msg.image.width / (float)msg.image.height, 1.0f);
    }

    private async Task<Texture2D> UpdateTexturePixels(int width, int height, byte[] data)
    {
        Texture2D tex = new Texture2D(width, height);
        for (int i = 0; i < width * height; i++)
        {
            int b = i * 3;

            byte blue = data[b];
            byte green = data[b + 1];
            byte red = data[b + 2];
            byte alpha = 255;

            int y = (int)(i / width);
            int x = (int)(i % width);

            tex.SetPixel(x, y, new Color32(red, green, blue, alpha));
        }
        return tex;
    }
    */

    /*
    private void UpdateCameraFeedCoroutineCall(RosSpotCameraImage msg)
    {
        IEnumerator feedCall = UpdateCameraFeedCoroutine(msg);
        StartCoroutine(feedCall);
    }

    private IEnumerator UpdateCameraFeedCoroutine(RosSpotCameraImage msg)
    {
        // Convert the response image to a texture
        Texture2D tex = new Texture2D((int)msg.image.width, (int)msg.image.height);
        for (int i = 0; i < msg.image.width * msg.image.height; i++)
        {
            int b = i * 3;

            byte blue = msg.image.data[b];
            byte green = msg.image.data[b + 1];
            byte red = msg.image.data[b + 2];
            byte alpha = 255;

            int y = (int)(i / (int)msg.image.width);
            int x = (int)(i % (int)msg.image.width);

            tex.SetPixel(x, y, new Color32(red, green, blue, alpha));
        }

        // Apply texture to a Unity canvas
        tex.Apply();
        cameraFeedImage.texture = tex;
        cameraNameText.text = cameraName;

        // Adjust canvas to match image dimensions
        cameraFeedImage.rectTransform.sizeDelta = new Vector2((float)msg.image.width / (float)msg.image.height, 1.0f);

        yield return null;
    }
    */

    private void BlankCameraFeed()
    {
        Texture2D tex = new Texture2D(1, 1);
        tex.SetPixel(0, 0, new Color32(1, 1, 1, 1));
        tex.Apply();
        cameraFeedImage.texture = tex;
        cameraNameText.text = cameraName;
        cameraFeedImage.rectTransform.sizeDelta = new Vector2(1.0f, 1.0f);
    }

    public void SetCamera(string cameraName)
    {
        // Remove current subscriptions to previous camera
        if(topic != "" && ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected)
            RemoveSubscribers(this, ros2Manager.GetROSConnection());
        BlankCameraFeed();

        imageData = null;
        imageHeight = 0;
        imageWidth = 0;

        // Update params
        this.cameraName = cameraName;
        topic = TOPIC_PREFIX + cameraName;

        // ROS2 has already been connected, so start camera stream using the connection
        if (ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected)
        {
            AddSubscribers(this, null);
        }
    }

    public void Close()
    {
        // Unsubscribe in ROS2
        if(ros2Manager.GetStatus() == ROS2Manager.ROS2ConnectionStatus.Connected)
            RemoveSubscribers(this, ros2Manager.GetROSConnection());
        // Clear memory
        if (tex != null)
        {
            Destroy(tex);
            tex = null;
        }
        // Remove base object
        GameObject.Destroy(transform.parent.gameObject);
    }

    private void AddSubscribers(object sender, EventArgs e)
    {
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null)
        {
            Debug.Log("Subscribing to: " + topic);
            ros.Subscribe<RosSpotCameraImage>(topic, SetImageData);
            //ros.Subscribe<RosSpotCameraImage>(topic, UpdateCameraFeed);
            //ros.Subscribe<RosSpotCameraImage>(topic, UpdateCameraFeedAsync);
            //ros.Subscribe<RosSpotCameraImage>(topic, UpdateCameraFeedCoroutineCall);
            ros2Manager.NotifyAboutSubscriberTopic(topic, true);
        }
    }

    private void RemoveSubscribers(object sender, ROSConnection ros)
    {
        Debug.Log("Unsubscribing to: " + topic);
        ros.Unsubscribe(topic);
        ros2Manager.NotifyAboutSubscriberTopic(topic, false);
    }

    public void RecieveGUIToggle(string cameraName)
    {
        /*
        // Untoggle all others
        foreach(var toggleInteractable in cameraViewToggles)
        {
            // Is pressed toggle
            if (toggleInteractable.transform.name.Contains(cameraName))
                continue;
            // Turn off
            toggleInteractable.SetToggled(false);
        }
        */

        // Select this camera
        SetCamera(cameraName);
    }
}
