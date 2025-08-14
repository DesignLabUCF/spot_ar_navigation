using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpotTransform
{
    //public readonly DateTime timestamp;
    public readonly DateTime timestamp;
    public readonly Vector3 position;
    public readonly Quaternion rotation;
    public readonly Vector3 linearVelocity;
    public readonly Vector3 angularVelocity;
    public readonly Vector3? aprilPosition;
    public readonly Quaternion? aprilRotation;
    public readonly List<SpotJoint> joints;

    public SpotTransform(DateTime timestamp, Vector3 position, Quaternion rotation, Vector3 linearVelocity, Vector3 angularVelocity, Vector3? aprilPosition, Quaternion? aprilRotation, List<SpotJoint> joints)
    {
        this.timestamp = timestamp;
        this.position = position;
        this.rotation = rotation;
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        this.aprilPosition = aprilPosition;
        this.aprilRotation = aprilRotation;
        this.joints = joints;
    }
}
