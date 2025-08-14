using UnityEngine;
//using Microsoft.MixedReality.QR;
using Microsoft.MixedReality.SENSEableQR;
using System;
using Microsoft.MixedReality.OpenXR;
using Unity.Robotics.ROSTCPConnector;

using RosSpotTransform = RosMessageTypes.UnityRoboticsDemo.SpotTransformMsg;
using RosSpotTagAndCamera = RosMessageTypes.UnityRoboticsDemo.TagAndCameraTransformMsg;
using UnityEngine.Experimental.XR.Interaction;
using Unity.XR.CoreUtils;

public class AnchorManager : MonoBehaviour
{
    [Tooltip("The data of the QR code that will sync the HoloLens 2 and Spot.")]
    public string SYNC_QR_ID = "Anchor1";

    private bool syncQRDetected = false;
    private UnityEngine.Pose syncQRPose;

    // ROS
    public ROS2Manager ros2Manager;
    //private ROSConnection ros;
    //private string topic = "hl_transform";
    private string topic = "hololens_tagandcamera";
    private float publishRate = 1.0f / 5.0f; // Publish FPS
    private float timeElapsed = 0; // Used to space out messages more    

    // Coordinate frame change reference stuff
    public GameObject unityCoordinateReferenceFrame;
    public GameObject unityCameraReference;

    public EventHandler validQRScanned;

    private void Awake()
    {

    }

    void Start()
    {
        GameObject.FindObjectOfType<QRCodesManager>().QRCodeAdded += NewQRCodeFound;
        GameObject.FindObjectOfType<QRCodesManager>().QRCodeUpdated += ExistingQRCodeFound;
        GameObject.FindObjectOfType<QRCodesManager>().QRCodeRemoved += ExistingQRCodeLost;

        //ros = ROSConnection.GetOrCreateInstance();
        //ros.RegisterPublisher<RosSpotTagAndCamera>(topic);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime; // Update time ticker

        /*
        if (timeElapsed > publishRate)
        {
            PublishHL2Pose(new UnityEngine.Pose(Vector3.up, Quaternion.identity), new UnityEngine.Pose(Vector3.left, Quaternion.identity)); // Sends out camera pose relative to the detected QR code
            timeElapsed = 0;
        }
        */

        if (SynchronizationQRHasBeenFound())
        {
            /*
            // QR
            UnityEngine.Pose qrPose = GetSyncQRPose();
            // HL2
            Vector3 cameraPosition = Camera.main.transform.position;
            Quaternion cameraRotation = Camera.main.transform.rotation;
            UnityEngine.Pose cameraPose = new UnityEngine.Pose(cameraPosition, cameraRotation);
            // Re-frame the HL2 camera from the QR position
            UnityEngine.Pose translatedPose = CoordinateConverter.HL2PoseFromQRPose(qrPose, cameraPose);
            Debug.Log(translatedPose.position);

            // Send to ROS2
            //Debug.Log("Translated: " + translatedPose);
            //Debug.Log("cameraPose: " + cameraPose);
            //Debug.Log("qrPose: " + qrPose);
            if (timeElapsed > publishRate)
            {
                PublishHL2Pose(cameraPose, qrPose); // Sends out camera pose relative to the detected QR code
                // Reset time ticker
                timeElapsed = 0;
            }
            */

            // QR
            UnityEngine.Pose qrPose = GetSyncQRPose();
            // HL2
            Vector3 cameraPosition = GetCameraPose().position;
            Quaternion cameraRotation = GetCameraPose().rotation;
            UnityEngine.Pose cameraPose = new UnityEngine.Pose(cameraPosition, cameraRotation);
            // Re-frame the HL2 camera from the QR position
            UnityEngine.Pose translatedPose = CoordinateConverter.HL2PoseFromQRPose(qrPose, cameraPose);

            // Update the coordinate frame change reference objects for later use
            unityCoordinateReferenceFrame.transform.SetWorldPose(qrPose);
            unityCameraReference.transform.SetWorldPose(cameraPose);

            // Convert coordinates from Spot geometry to Unity
            //translatedPose = CoordinateConverter.UnityToSpot(translatedPose); // TODO have it send out this instead

            // Send to ROS2
            //Debug.Log("Translated: " + translatedPose);
            //Debug.Log("cameraPose: " + cameraPose);
            //Debug.Log("qrPose: " + qrPose);
            if (timeElapsed > publishRate)
            {
                PublishHL2Pose(CoordinateConverter.UnityToSpot(cameraPose), CoordinateConverter.UnityToSpot(qrPose)); // Sends out camera pose relative to the detected QR code
                // Reset time ticker
                timeElapsed = 0;
            }
        }
    }

    public Pose GetCameraPose()
    {
        return new Pose(Camera.main.transform.position, Camera.main.transform.rotation);
    }

    private void NewQRCodeFound(object sender, QRCodeEventArgs<Microsoft.MixedReality.QR.QRCode> e)
    {
        string qrData = e.Data.Data;
        Guid qrGUID = e.Data.SpatialGraphNodeId;

        // QR code for syncing between Spot and HL2
        if (qrData == SYNC_QR_ID)
        {
            syncQRDetected = true;
            syncQRPose = GetQRPose(qrGUID);
            validQRScanned.Invoke(this, null);
        }
    }

    private void ExistingQRCodeFound(object sender, QRCodeEventArgs<Microsoft.MixedReality.QR.QRCode> e)
    {
        string qrData = e.Data.Data;
        Guid qrGUID = e.Data.SpatialGraphNodeId;

        // QR code for syncing between Spot and HL2
        if (qrData == SYNC_QR_ID)
        {
            syncQRDetected = true;
            syncQRPose = GetQRPose(qrGUID);
            validQRScanned.Invoke(this, null);
        }
    }

    private void ExistingQRCodeLost(object sender, QRCodeEventArgs<Microsoft.MixedReality.QR.QRCode> e)
    {
        string qrData = e.Data.Data;
    }

    private UnityEngine.Pose GetQRPose(Guid guid)
    {
        SpatialGraphNode node = SpatialGraphNode.FromStaticNodeId(guid);
        node.TryLocate(FrameTime.OnUpdate, out UnityEngine.Pose pose);

        /*
        if (CameraCache.Main.transform.parent != null)
        {
            pose = pose.GetTransformedBy(CameraCache.Main.transform.parent);
        }
        */

        //Debug.Log(pose.position);
        //Debug.Log(pose.rotation);
        return pose;
    }

    public void PublishHL2Pose(UnityEngine.Pose hl2Pose, UnityEngine.Pose qrPose)
    {
        /*
        cube.transform.rotation = Random.rotation;

        PosRotMsg cubePos = new PosRotMsg(
            cube.transform.position.x,
            cube.transform.position.y,
            cube.transform.position.z,
            cube.transform.rotation.x,
            cube.transform.rotation.y,
            cube.transform.rotation.z,
            cube.transform.rotation.w
        );

        // Finally send the message to server_endpoint.py running in ROS
        ros.Publish(topic, cubePos);
        */

        // Create messages
        RosSpotTransform hl2Transform = new RosSpotTransform(
            hl2Pose.position.x,
            hl2Pose.position.y,
            hl2Pose.position.z,
            hl2Pose.rotation.x,
            hl2Pose.rotation.y,
            hl2Pose.rotation.z,
            hl2Pose.rotation.w,
            1.0f,
            1.0f,
            1.0f
            );
        RosSpotTransform qrTransform = new RosSpotTransform(
            qrPose.position.x,
            qrPose.position.y,
            qrPose.position.z,
            qrPose.rotation.x,
            qrPose.rotation.y,
            qrPose.rotation.z,
            qrPose.rotation.w,
            1.0f,
            1.0f,
            1.0f
            );

        RosSpotTagAndCamera tagAndCameraTransform = new RosSpotTagAndCamera(qrTransform, hl2Transform);

        // Publish
        //Debug.Log("Publishing to topic: " + topic);
        //ros.Publish(topic, tagAndCameraTransform);
        ROSConnection ros = ros2Manager.GetROSConnection();
        if(ros != null)
        {
            // Register the topic
            if (ros2Manager.ShouldRegisterTopic(topic))
            {
                ros.RegisterPublisher<RosSpotTagAndCamera>(topic);
                ros2Manager.NotifyAboutPublishingTopic(topic, true);
            }
            // Send message over topic
            ros.Publish(topic, tagAndCameraTransform);
        }
    }

    public bool SynchronizationQRHasBeenFound()
    {
        return syncQRDetected;
    }

    public UnityEngine.Pose GetSyncQRPose()
    {
        return syncQRPose;
    }

    public Vector3 GetSyncQRPosition()
    {
        return syncQRPose.position;
    }

    public Quaternion GetSyncQRRotation()
    {
        return syncQRPose.rotation;
    }
}
