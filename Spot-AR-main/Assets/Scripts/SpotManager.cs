using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Assertions;
using Unity.XR.CoreUtils;

using RosSpotJoint = RosMessageTypes.UnityRoboticsDemo.SpotJointMsg;
using RosSpotJoints = RosMessageTypes.UnityRoboticsDemo.SpotJointsMsg;
using RosSpotTransform = RosMessageTypes.UnityRoboticsDemo.SpotTransformMsg;
using RosSpotTagAndCamera = RosMessageTypes.UnityRoboticsDemo.TagAndCameraTransformMsg;
using System.Runtime.Remoting.Messaging;

public class SpotManager : MonoBehaviour
{
    //public ROS2Subscriber ros2Subscriber;
    public AnchorManager anchorManager;
    public ROS2Manager ros2Manager;

    public GameObject spotModel;
    public GameObject aprilTagModel;
    private bool spotModelVisibile = true;
    private bool spotSendingAllNecessaryData = false;

    private Vector3 spotPosition;
    private Quaternion spotRotation;
    private Vector3 spotVelocity;
    public Pose mostRecentSpotInBDCoordinates;
    public Pose mostRecentAprilTagInBDCoodinates;
    private Dictionary<string, SpotJoint> joints;

    private string cameraAndTagTopic = "spot_tagandcamera";
    private string jointsTopic = "joints";

    // Coordinate frame change reference stuff
    public GameObject bdCoordinateReferenceFrame;
    public GameObject bdCameraReference;

    public EventHandler validSpotJointsReceived;
    public EventHandler validSpotTransformReceived;
    public EventHandler validSpotAprilTagReceived;


    private void Awake()
    {
        if (anchorManager == null)
            throw new Exception("AnchorManager not set on SpotManager object");

        joints = new Dictionary<string, SpotJoint>
        {
            {"fl.hx", new SpotJoint("fl.hx")},
            {"fl.hy", new SpotJoint("fl.hy")},
            {"fl.kn", new SpotJoint("fl.kn")},
            {"fr.hx", new SpotJoint("fr.hx")},
            {"fr.hy", new SpotJoint("fr.hy")},
            {"fr.kn", new SpotJoint("fr.kn")},
            {"hl.hx", new SpotJoint("hl.hx")},
            {"hl.hy", new SpotJoint("hl.hy")},
            {"hl.kn", new SpotJoint("hl.kn")},
            {"hr.hx", new SpotJoint("hr.hx")},
            {"hr.hy", new SpotJoint("hr.hy")},
            {"hr.kn", new SpotJoint("hr.kn")}
        };
    }


    void Start()
    {
        SetSpotVisibility(false);

        ros2Manager.rosConnectedEvent += AddSubscribers;
        ros2Manager.rosDisconnectedEvent += RemoveSubscribers;
    }

    void Update()
    {

    }

    private void AddSubscribers(object sender, EventArgs e)
    {
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null)
        {
            Debug.Log("Subscribing to: " + cameraAndTagTopic);
            ros.Subscribe<RosSpotTagAndCamera>(cameraAndTagTopic, RecieveTransformMsg);
            ros2Manager.NotifyAboutSubscriberTopic(cameraAndTagTopic, true);
            Debug.Log("Subscribing to: " + jointsTopic);
            ros.Subscribe<RosSpotJoints>(jointsTopic, RecieveJointsMsg);
            ros2Manager.NotifyAboutSubscriberTopic(jointsTopic, true);
        }
    }

    private void RemoveSubscribers(object sender, ROSConnection ros)
    {
        Debug.Log("Unsubscribing to: " + cameraAndTagTopic);
        ros.Unsubscribe(cameraAndTagTopic);
        ros2Manager.NotifyAboutSubscriberTopic(cameraAndTagTopic, false);
        Debug.Log("Unsubscribing to: " + jointsTopic);
        ros.Unsubscribe(jointsTopic);
        ros2Manager.NotifyAboutSubscriberTopic(jointsTopic, false);

        // Update internal values
        spotSendingAllNecessaryData = false;
        SetSpotVisibility(false);
    }

    void RecieveTransformMsg(RosSpotTagAndCamera message)
    {
        //Debug.Log("Received spot transform: " + spotTransform.ToString());

        // Show Spot if it was hidden and this is the first message received
        if (ShouldShowSpot())
        {
            SetSpotVisibility(true);
        }

        validSpotTransformReceived.Invoke(this, null);
        if (SpotAprilTagReadingIsValid(message.tag))
        {
            spotSendingAllNecessaryData = true;
            validSpotAprilTagReceived.Invoke(this, null);
}

        // Update the coordinate frame change reference objects for later use

        mostRecentSpotInBDCoordinates = new Pose(new Vector3(message.camera.pos_x, message.camera.pos_y, message.camera.pos_z), new Quaternion(message.camera.rot_x, message.camera.rot_y, message.camera.rot_z, message.camera.rot_w));
        mostRecentAprilTagInBDCoodinates = new Pose(new Vector3(message.tag.pos_x, message.tag.pos_y, message.tag.pos_z), new Quaternion(message.tag.rot_x, message.tag.rot_y, message.tag.rot_z, message.tag.rot_w));
        bdCoordinateReferenceFrame.transform.SetWorldPose(mostRecentAprilTagInBDCoodinates);
        bdCameraReference.transform.SetWorldPose(mostRecentSpotInBDCoordinates);

        // Translate models to Unity coordinates
        UnityEngine.Pose[] unityCoordinatePoses = UpdateSpotAndTagModels(message.tag, message.camera);

        // Translate the models to be relative to the HoloLens scanned fiducial
        TranslateSpotAndTagModels(unityCoordinatePoses[0], unityCoordinatePoses[1]);
    }

    private UnityEngine.Pose[] UpdateSpotAndTagModels(RosSpotTransform tagTransform, RosSpotTransform spotTransform)
    {

        // Translate the message into Unity form - April tag
        Vector3 aprilTagPos = new Vector3(tagTransform.pos_x, tagTransform.pos_y, tagTransform.pos_z);
        Quaternion aprilTagRot = new Quaternion(tagTransform.rot_x, tagTransform.rot_y, tagTransform.rot_z, tagTransform.rot_w);
        //Vector3 aprilTagRot = new Vector3(aprilTagTransform.rot_x, aprilTagTransform.rot_y, aprilTagTransform.rot_z);
        UnityEngine.Pose aprilPose = new UnityEngine.Pose(aprilTagPos, aprilTagRot);

        // Translate the message into Unity form - Spot body (vision frame)
        Vector3 spotPos = new Vector3(spotTransform.pos_x, spotTransform.pos_y, spotTransform.pos_z);
        Quaternion spotRot = new Quaternion(spotTransform.rot_x, spotTransform.rot_y, spotTransform.rot_z, spotTransform.rot_w);
        //Vector3 spotRot = new Vector3(spotTransform.rot_x, spotTransform.rot_y, spotTransform.rot_z);
        UnityEngine.Pose spotPose = new UnityEngine.Pose(spotPos, spotRot);

        // Get spot's relative position from the april tag
        UnityEngine.Pose translatedPose = CoordinateConverter.SpotPoseFromAprilPose(aprilPose, spotPose);

        // Convert coordinates from Spot geometry to Unity
        aprilPose = CoordinateConverter.SpotToUnity(aprilPose);
        spotPose = CoordinateConverter.SpotToUnity(spotPose);
        //translatedPose = CoordinateConverter.SpotToUnity(translatedPose);

        // Update april tag model 
        aprilTagModel.transform.localPosition = aprilPose.position;
        aprilTagModel.transform.localRotation = aprilPose.rotation;
        //Debug.Log("Received april tag transform: " + aprilTagTransform.ToString());

        // Update Spot model
        //spotModel.transform.localPosition = translatedPose.position;
        //spotModel.transform.localRotation = translatedPose.rotation;
        spotModel.transform.localPosition = spotPose.position;
        spotModel.transform.localRotation = spotPose.rotation;

        // Return back the Unity coordinate poses
        return new UnityEngine.Pose[2] { aprilPose, spotPose };
    }

    // Translates the Converted spot coordinates to the Unity-scanned fiducial
    private void TranslateSpotAndTagModels(UnityEngine.Pose aprilPose, UnityEngine.Pose spotPose)
    {
        UnityEngine.Pose holoLensPose = anchorManager.GetCameraPose();
        UnityEngine.Pose holoLensFiducialPose = anchorManager.GetSyncQRPose();

        // Position
        Vector3 posTranslation = holoLensFiducialPose.position - aprilPose.position;
        aprilPose.position = aprilPose.position + posTranslation;
        aprilTagModel.transform.localPosition = aprilPose.position;
        spotPose.position = spotPose.position + posTranslation;
        spotModel.transform.localPosition = spotPose.position;

        // Rotation
        spotModel.transform.parent = aprilTagModel.transform;
        Quaternion rotTranslation = holoLensFiducialPose.rotation * Quaternion.Inverse(aprilPose.rotation);
        aprilPose.rotation = rotTranslation * aprilPose.rotation;
        aprilPose.rotation = aprilPose.rotation * Quaternion.Euler(90.0f, 0, 0);
        aprilTagModel.transform.localRotation = aprilPose.rotation;
        spotModel.transform.parent = null; // TODO reapply original parent
    }

    /*
    void RecieveJointMsg(RosSpotJoint message)
    {
        // Show Spot if it was hidden and this is the first message received
        if(ShouldShowSpot())
        {
            SetSpotVisibility(true);
        }

        // Make sure the joint name is valid
        SpotJoint joint = joints[message.name];
        Assert.IsNotNull(joint, "Joint '" + message.name + "' received from ROS2 not found in joints dict.");
        // Update Spot model
        SetModelJoint(message.name, message.position);
    }
    */
    void RecieveJointsMsg(RosSpotJoints message)
    {
        // Show Spot if it was hidden and this is the first message received
        if (ShouldShowSpot())
        {
            SetSpotVisibility(true);
        }

        validSpotJointsReceived.Invoke(this, null);

        // Update each joint from the message joint array
        foreach (RosSpotJoint msg_joint in message.joints)
        {
            // Make sure the joint name is valid
            SpotJoint joint = joints[msg_joint.name];
            Assert.IsNotNull(joint, "Joint '" + msg_joint.name + "' received from ROS2 not found in joints dict.");
            // Update Spot model
            SetModelJoint(msg_joint.name, msg_joint.position);
        }
    }

    private void SetModelJoint(string spotJointName, float position)
    {
        //Debug.Log("Updating joint '" +  spotJointName + "' to position: " +  position);
        // Update internal joint value
        joints[spotJointName].SetPosition(position);
        // Get proper value to set to after translating between BD and Unity needs
        Vector3 rot = SpotJoint.JointPositionToEulerAngles(joints[spotJointName]);
        // Set model rotation
        GameObject modelJoint = SpotJoint.GetJointGameObject(spotJointName);
        //Debug.Log("Setting joint '" + modelJoint.name + "' to value " + rot);
        if(modelJoint == null)
        {
            Debug.Log("Joint model unable to be found. Exiting SetModelJoint early...");
            return;
        }
        modelJoint.transform.localEulerAngles = rot;
    }

    private void SetSpotVisibility(bool visibility)
    {
        spotModelVisibile = visibility;
        spotModel.SetActive(visibility);
        aprilTagModel.SetActive(visibility);
    }

    private bool ShouldShowSpot()
    {
        bool spotStillHidden = spotModelVisibile == false;
        bool hl2IsPublishing = anchorManager.SynchronizationQRHasBeenFound();
        return (spotStillHidden && hl2IsPublishing);

        //return spotModelVisibile == false;
    }

    private bool SpotAprilTagReadingIsValid(RosSpotTransform tag)
    {
        if (tag.rot_x == -100.0f && tag.rot_y == -100.0f && tag.rot_z == -100.0f && tag.rot_w == -100.0f) // Default values we set in spot_transform_publisher.py
            return false;
        else
            return true;
    }

    public bool SpotIsPublishingWithValidAprilTag()
    {
        return spotSendingAllNecessaryData;
    }
}
