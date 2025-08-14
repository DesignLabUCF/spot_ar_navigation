using UnityEngine;
using System;

public class CoordinateConverter : MonoBehaviour
{
    public AnchorManager anchorManager;

    private void Awake()
    {
        if (anchorManager == null)
            throw new Exception("AnchorManager not set on CoordinateConverter object");
    }

    void Start()
    {
        
    }

    void Update()
    {

    }

    public static  UnityEngine.Pose HL2PoseFromQRPose(UnityEngine.Pose qrPose, UnityEngine.Pose cameraPose)
    {
        return cameraPose.GetTransformedBy(qrPose); // TODO
    }

    public static UnityEngine.Pose SpotPoseFromAprilPose(UnityEngine.Pose aprilPose, UnityEngine.Pose spotPose)
    {
        return spotPose.GetTransformedBy(aprilPose); // TODO
    }

    public static UnityEngine.Pose SpotToUnity(UnityEngine.Pose spotRelativeToAprilTag)
    {
        /*
        // Rotate Z 180 degrees
        spotRelativeToAprilTag = new Pose(spotRelativeToAprilTag.position, Quaternion.FromToRotation(spotRelativeToAprilTag.rotation.eulerAngles, spotRelativeToAprilTag.rotation.eulerAngles + new Vector3(0f, 0f, 180f)));
        // Change axes
        Vector3 newRotation = spotRelativeToAprilTag.rotation.eulerAngles;
        float tempZ = newRotation.z;
        newRotation.z = newRotation.y;
        newRotation.y = newRotation.x;
        newRotation.x = tempZ;
        return new Pose(spotRelativeToAprilTag.position, Quaternion.Euler(newRotation));
        */

        /*
        // Rotate Z 180 degrees
        Vector3 newPosition = spotRelativeToAprilTag.position.RotateAround(Vector3.zero, Quaternion.Euler(0f, 0f, 180));
        Quaternion newRotation = spotRelativeToAprilTag.rotation * Quaternion.Euler(Vector3.forward * 180);
        // Change axes
        float tempZ = newPosition.z;
        newPosition.z = newPosition.y;
        newPosition.y = newPosition.x;
        newPosition.x = tempZ;
        Vector3 newRotationVec = newRotation.eulerAngles;
        tempZ = newRotationVec.z;
        newRotationVec.z = newRotationVec.y;
        newRotationVec.y = newRotationVec.x;
        newRotationVec.x = tempZ;
        return new Pose(newPosition, Quaternion.Euler(newRotationVec));
        */

        /*
        spotRelativeToAprilTag.rotation = Quaternion.Euler(spotRelativeToAprilTag.rotation.eulerAngles + new Vector3(90f, 0f, 0f));
        spotRelativeToAprilTag.rotation = Quaternion.Euler(spotRelativeToAprilTag.rotation.eulerAngles + new Vector3(0f, 0f, 180f));
        */

        // Kider solution
        /*
        // negative 90z
        // 90 x
        // 90 y
        spotRelativeToAprilTag.rotation = Quaternion.Euler(spotRelativeToAprilTag.rotation.eulerAngles + new Vector3(0f, 0f, -90f));
        spotRelativeToAprilTag.rotation = Quaternion.Euler(spotRelativeToAprilTag.rotation.eulerAngles + new Vector3(90f, 0f, 0f));
        spotRelativeToAprilTag.rotation = Quaternion.Euler(spotRelativeToAprilTag.rotation.eulerAngles + new Vector3(0f, 90f, 0f));

        return spotRelativeToAprilTag; // TODO
        */

        /*
        spotRelativeToAprilTag.rotation = Quaternion.identity *
            Quaternion.AngleAxis(spotRelativeToAprilTag.rotation.eulerAngles.z, -Vector3.up) *
            Quaternion.AngleAxis(spotRelativeToAprilTag.rotation.eulerAngles.y, Vector3.forward) *
            Quaternion.AngleAxis(spotRelativeToAprilTag.rotation.eulerAngles.x, Vector3.right);
        return spotRelativeToAprilTag;
        */

        // Found by trial and error in PlaybackManager
        Vector3 convertedPos = new Vector3(spotRelativeToAprilTag.position.x, spotRelativeToAprilTag.position.z, spotRelativeToAprilTag.position.y);
        Quaternion convertedRot = new Quaternion(-spotRelativeToAprilTag.rotation.x, -spotRelativeToAprilTag.rotation.z, -spotRelativeToAprilTag.rotation.y, spotRelativeToAprilTag.rotation.w);
        convertedRot = convertedRot * Quaternion.Euler(0, 90.0f, 0); // Rotate quaternion additional 90 degrees around Y axis
        return new UnityEngine.Pose(convertedPos, convertedRot);
    }

    public static UnityEngine.Pose UnityToSpot(UnityEngine.Pose cameraRelativeToQR)
    {
        // TODO, not sure if below works. Probably not.

        /*
        cameraRelativeToQR.rotation = Quaternion.Euler(cameraRelativeToQR.rotation.eulerAngles + new Vector3(0f, 90f, 0f));
        cameraRelativeToQR.rotation = Quaternion.Euler(cameraRelativeToQR.rotation.eulerAngles + new Vector3(90f, 0f, 0f));
        cameraRelativeToQR.rotation = Quaternion.Euler(cameraRelativeToQR.rotation.eulerAngles + new Vector3(0f, 0f, -90f));
        */

        Vector3 convertedPos = new Vector3(cameraRelativeToQR.position.x, cameraRelativeToQR.position.z, cameraRelativeToQR.position.y);
        Quaternion convertedRot = new Quaternion(-cameraRelativeToQR.rotation.x, -cameraRelativeToQR.rotation.z, -cameraRelativeToQR.rotation.y, cameraRelativeToQR.rotation.w);
        convertedRot = convertedRot * Quaternion.Euler(0, -90.0f, 0); // Rotate quaternion additional 90 degrees around Y axis

        return new UnityEngine.Pose(convertedPos, convertedRot);
    }

}
