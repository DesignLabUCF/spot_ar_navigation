using Microsoft.MixedReality.OpenXR.Remoting;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Experimental.Joystick;
using Microsoft.MixedReality.Toolkit.Input;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting;
using Unity.XR.CoreUtils;
using UnityEngine;

using RosVelocity = RosMessageTypes.UnityRoboticsDemo.SpotVelocityMsg;

public class VelocityManager : MonoBehaviour
{
    public JoystickController_SENSEable joystick;
    public ROS2Manager ros2Manager;
    public AnchorManager anchorManager;
    public SpotManager spotManager;
    public ParticipantLogger participantLogger;

    public GameObject bdPointGoToReference;
    public GameObject unityPointGoToReference;

    // ROS
    // private ROSConnection ros;
    private string topic = "velocity";
    private float publishRate = 1.0f / 30.0f; // Publish FPS
    public float timeElapsed = 0; // Used to space out messages more    

    [Range(0.1f, 3.0f)]
    public float positionMultiplier = 1.0f;
    [Range(0.1f, 3.0f)]
    public float rotationMultiplier = 1.0f;

    // Direct velocity controller
    private float v_x = 0;
    private float v_y = 0;
    private float v_rot = 0;
    // Point controls
    private float x_point = 0;
    private float y_point = 0;
    private float z_point = 0;
    private bool invert_heading = false;
    private bool usePointControls = true;
    private bool movementIsStopped = false;

    public EventHandler<string> controlTypeChanged;
    public EventHandler<string> invertHeadingChanged;
    public EventHandler pointCommandIssueFailed;
    public EventHandler<Vector3[]> spotDestinationSent;
    public EventHandler spotDestinationCancel;
    public EventHandler<bool> movementStopStatusChanged;

    public bool Invert_heading { get => invert_heading; }
    public bool UsePointControls { get => usePointControls; }
    public bool MovementIsStopped { get => movementIsStopped; }

    private void Awake()
    {

    }

    // Start is called before the first frame update
    void Start()
    {
        //ros = ROSConnection.GetOrCreateInstance();
        //ros.RegisterPublisher<RosVelocity>(topic);

        //Invoke("InitSettings", 3.0f); // Initalize after delay to make sure events have been properly registered in other classes
    }

    // Update is called once per frame
    void Update()
    {
        // Reset params
        v_x = 0;
        v_y = 0;
        v_rot = 0;
        x_point = 0;
        y_point = 0;
        z_point = 0;

        // Our psuedo-estop is in effect
        if(movementIsStopped == true)
        {
            return;
        }

        // Read joystick input
        if (usePointControls == false)
        {
            // Get velocity input
            /*
            v_x = 0;
            v_y = 0;
            v_rot = 0;
            if(Input.GetKey(KeyCode.W)) // Debug test
            {
                v_x = 1.0f;
            }
            if (Input.GetKey(KeyCode.S)) // Debug test
            {
                v_x = -1.0f;
            }
            if (Input.GetKey(KeyCode.A)) // Debug test
            {
                v_y = 1.0f;
            }
            if (Input.GetKey(KeyCode.D)) // Debug test
            {
                v_y = -1.0f;
            }
            if (Input.GetKey(KeyCode.Q)) // Debug test
            {
                v_rot = 1.0f;
            }
            if (Input.GetKey(KeyCode.E)) // Debug test
            {
                v_rot = -1.0f;
            }
            */

            // Pipe joystick input to velocity values
            var joystickMove = joystick.GetJoyStickMove();
            if (Mathf.Abs(joystickMove.x) > 0.1)
                v_x = joystickMove.x;
            if (Mathf.Abs(joystickMove.z) > 0.1)
                v_y = joystickMove.z;
            var joystickRotation = joystick.GetJoyStickRotation();
            if (Mathf.Abs(joystickRotation) >= 5.0f)
            {
                if (joystickRotation >= 0)
                {
                    v_rot = joystickRotation / 90.0f;
                }
                else
                {
                    v_rot = joystickRotation / 90.0f;
                }
                v_rot = -v_rot;
            }
            // Amplify values
            v_x = v_x * positionMultiplier;
            v_y = v_y * positionMultiplier;
            v_rot = v_rot * rotationMultiplier;
        }
        else
        {
            ; // TODO
        }

        // Send input to ROS
        timeElapsed += Time.deltaTime; // Update time ticker
        if (timeElapsed > publishRate)
        {
            if (usePointControls == false)
                AttemptPublishVelocity(v_x, v_y, v_rot);
            else
                //AttemptPublishPoint(x_point, y_point, z_point);
                ;
            timeElapsed = 0; // Reset time ticker
        }
    }

    /*
    private void InitSettings()
    {
        SetControlType(true);
    }
    */

    public void ReceiveStopInput()
    {
        if(movementIsStopped == true)
            ResumeMovement();
        else
            StopAllMovement();
    }

    public void StopAllMovement()
    {
        movementIsStopped = true;
        AttemptPublishVelocity(0, 0, 0);
        movementStopStatusChanged.Invoke(this, true);
        participantLogger.AddStop(true);
    }

    public void ResumeMovement()
    {
        movementIsStopped = false;
        movementStopStatusChanged.Invoke(this, false);
        participantLogger.AddStop(false);
    }

    private void AttemptPublishVelocity(float x, float y, float rot)
    {
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null) // ROS has connected
        {
            // Register the topic
            if (ros2Manager.ShouldRegisterTopic(topic))
            {
                ros.RegisterPublisher<RosVelocity>(topic);
                ros2Manager.NotifyAboutPublishingTopic(topic, true);
            }
            // Send message over topic
            //RosVelocity velocityMsg = new RosVelocity(x, y, rot, false, 0, 0, 0, 0, 0, 0, 0, 0);
            RosVelocity velocityMsg = new RosVelocity("velocity", x, y, rot, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, invert_heading);
            ros.Publish(topic, velocityMsg);
        }
    }

    private void AttemptPublishPoint(bool isRelative, float x, float y, float z)
    {

        //spotDestinationSent.Invoke(this, CoordinateConverter.SpotToUnity(new Pose(new Vector3(x, y, z), Quaternion.identity)).position);

        // Define command parameters
        string commandType = "";
        float v_x = 0;
        float v_y = 0;
        float v_rot = 0;
        float absolute_x = 0;
        float absolute_y = 0;
        float absolute_z = 0;
        float offset_x = 0;
        float offset_y = 0;
        float april_pos_x = 0;
        float april_pos_y = 0;
        float april_rot_x = 0;
        float april_rot_y = 0;
        float april_rot_z = 0;
        float april_rot_w = 0;
        if (isRelative)
        {
            commandType = "relative_point";
            v_x = 0;
            v_y = 0;
            v_rot = 0;
            absolute_x = 0;
            absolute_y = 0;
            absolute_z = 0;
            offset_x = x;
            offset_y = y;
            april_pos_x = spotManager.mostRecentAprilTagInBDCoodinates.position.x;
            april_pos_y = spotManager.mostRecentAprilTagInBDCoodinates.position.y;
            april_rot_x = spotManager.mostRecentAprilTagInBDCoodinates.rotation.x;
            april_rot_y = spotManager.mostRecentAprilTagInBDCoodinates.rotation.y;
            april_rot_z = spotManager.mostRecentAprilTagInBDCoodinates.rotation.z;
            april_rot_w = spotManager.mostRecentAprilTagInBDCoodinates.rotation.w;
        }
        else
        {
            commandType = "absolute_point";
            v_x = 0;
            v_y = 0;
            v_rot = 0;
            absolute_x = x;
            absolute_y = y;
            absolute_z = z;
            offset_x = 0;
            offset_y = 0;
            april_pos_x = 0;
            april_pos_y = 0;
            april_rot_x = 0;
            april_rot_y = 0;
            april_rot_z = 0;
            april_rot_w = 0;
        }

        // Issue command
        ROSConnection ros = ros2Manager.GetROSConnection();
        if (ros != null) // ROS has connected
        {
            // Register the topic
            if (ros2Manager.ShouldRegisterTopic(topic))
            {
                ros.RegisterPublisher<RosVelocity>(topic);
                ros2Manager.NotifyAboutPublishingTopic(topic, true);
            }
            // Send message over topic
            Debug.Log("Publishing goal point: " + new Vector3(x, y, z).ToString());
            RosVelocity velocityMsg = new RosVelocity(commandType, v_x, v_y, v_rot, absolute_x, absolute_y, absolute_z, offset_x, offset_y, april_pos_x, april_pos_y, april_rot_x, april_rot_y, april_rot_z, april_rot_w, invert_heading);
            //RosVelocity velocityMsg = new RosVelocity(0, 0, 0, true, x, y, spotManager.mostRecentAprilTagInBDCoodinates.position.x, spotManager.mostRecentAprilTagInBDCoodinates.position.y, spotManager.mostRecentAprilTagInBDCoodinates.rotation.x, spotManager.mostRecentAprilTagInBDCoodinates.rotation.y, spotManager.mostRecentAprilTagInBDCoodinates.rotation.z, spotManager.mostRecentAprilTagInBDCoodinates.rotation.w);
            ros.Publish(topic, velocityMsg);
        }
    }
    private void AttemptPublishPoint(bool isRelative, Vector3 point)
    {
        AttemptPublishPoint(isRelative, point.x, point.y, point.z);
    }

    private bool ShouldPublishPointCommand()
    {
        // HoloLens is publishing, Spot is publishing, and our psuedo-estop is not in effect
        return usePointControls && anchorManager.SynchronizationQRHasBeenFound() && spotManager.SpotIsPublishingWithValidAprilTag() && !movementIsStopped;
    }

    private void PointPublishUnavailable()
    {
        pointCommandIssueFailed.Invoke(this, null);
    }

    // Tell Spot to come to the camera's position.
    public void ComeHereCommand()
    {
        //SetControlType(true);
        if (ShouldPublishPointCommand())
        {
            /*
            Vector3 spotFiducial = spotManager.mostRecentAprilTagInBDCoodinates.position;
            Vector3 unityFiducial = anchorManager.GetSyncQRPosition();
            Vector3 cameraPosition = Camera.main.transform.position;


            Vector3 cameraReferenceUnity = anchorManager.unityCameraReference.transform.localPosition;


            bdPointGoToReference.transform.localPosition = cameraReferenceUnity; // May need to change axes/negate next
            bdPointGoToReference.transform.localPosition = new Vector3(-bdPointGoToReference.transform.localPosition.x, bdPointGoToReference.transform.localPosition.y, bdPointGoToReference.transform.localPosition.z);
            Debug.Log("BdRef-local/Camera-ref: " + bdPointGoToReference.transform.localPosition);
            Debug.Log("BdRef-global: " + bdPointGoToReference.transform.position);
            Vector3 goal = bdPointGoToReference.transform.position;
            goal = new Vector3(goal.z, goal.y, goal.x);



            Vector3 unityDifference = cameraPosition - unityFiducial;
            Vector3[] markerPoints = { unityFiducial, unityFiducial + unityDifference };
            spotDestinationSent.Invoke(this, markerPoints);
            //goalPosition = CoordinateConverter.UnityToSpot(new Pose(goalPosition, Quaternion.identity)).position;
            AttemptPublishPoint(goal);
            */


            // Set up reference points to get relative position to QR in both coordinate frames
            unityPointGoToReference.transform.SetWorldPose(new Pose(Camera.main.transform.position, Quaternion.identity));
            Vector3 unityRelativeMetersToTag = unityPointGoToReference.transform.localPosition;
            Vector3 goal = new Vector3(unityRelativeMetersToTag.z, unityRelativeMetersToTag.x, 0);

            // Update visual marker for user reference
            Vector3[] markerPoints = { anchorManager.GetSyncQRPosition(), unityPointGoToReference.transform.position };
            spotDestinationSent.Invoke(this, markerPoints);

            // Publish point over ROS2
            AttemptPublishPoint(true, goal);
        }
        else
        {
            PointPublishUnavailable();
        }
    }

    public void GoToOriginCommand()
    {
        //SetControlType(true);
        if (ShouldPublishPointCommand())
        {
            AttemptPublishPoint(false, Vector3.zero);
        }
        else
        {
            PointPublishUnavailable();
        }
    }

    public bool GoToPointCommand(float x, float y, float z)
    {
        //SetControlType(true);
        if (ShouldPublishPointCommand())
        {
            // Set up reference points to get relative position to QR in both coordinate frames
            unityPointGoToReference.transform.SetWorldPose(new Pose(new Vector3(x, y, z), Quaternion.identity));
            Vector3 unityRelativeMetersToTag = unityPointGoToReference.transform.localPosition;
            Vector3 goal = new Vector3(unityRelativeMetersToTag.z, unityRelativeMetersToTag.x, 0);

            // Update visual marker for user reference
            Vector3[] markerPoints = { anchorManager.GetSyncQRPosition(), unityPointGoToReference.transform.position };
            spotDestinationSent.Invoke(this, markerPoints);

            // Save this study data
            participantLogger.AddGoToPoint(unityRelativeMetersToTag, invert_heading);

            // Publish point over ROS2
            AttemptPublishPoint(true, goal);
            return true;
        }
        else
        {
            PointPublishUnavailable();
            return false;
        }
    }

    public void SetControlType(bool usePointControls)
    {
        this.usePointControls = usePointControls;
        controlTypeChanged.Invoke(this, usePointControls.ToString());
    }

    public void ToggleControlType()
    {
        SetControlType(!usePointControls);
    }

    public void SetInvertHeading(bool invert_heading)
    {
        this.invert_heading = invert_heading;
        invertHeadingChanged.Invoke(this, invert_heading.ToString());
    }

    public void ToggleInvertHeading()
    {
        SetInvertHeading(!invert_heading);
    }

    #region Button controls
    public void SetXVelocity(float vel)
    {
        v_x = vel;
    }

    public void SetYVelocity(float vel)
    {
        v_y = vel;
    }

    public void SetRotationVelocity(float rot)
    {
        v_rot = rot;
    }

    public void Forward()
    {
        v_x = 1.0f;
    }

    public void Backward()
    {
        v_x = -1.0f;
    }

    public void StrafeLeft()
    {
        v_y = 1.0f;
    }

    public void StrafeRight()
    {
        v_y = -1.0f;
    }

    public void TurnLeft()
    {
        v_rot = 1.0f;
    }

    public void TurnRight()
    {
        v_rot = -1.0f;
    }

    public void ResetX()
    {
        v_x = 0f;
    }

    public void ResetY()
    {
        v_y = 0f;
    }

    public void ResetRotation()
    {
        v_rot = 0f;
    }
    #endregion
}
