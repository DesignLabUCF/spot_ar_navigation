using System.Collections.Generic;
using UnityEngine;

public class SpotJoint
{
    public string name = "";
    public float position = -1.0f;
    public float velocity = -1.0f;
    public float acceleration = -1.0f;
    public float load = -1.0f;

    public SpotJoint(string name, float position, float velocity, float acceleration, float load)
    {
        this.name = name;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.load = load;
    }
    public SpotJoint(string name)
    {
        this.name = name;
    }

    public float SetPosition(float position)
    {
        this.position = position;
        return this.position;
    }

    public static readonly Dictionary<string, string> jointNameMap = new Dictionary<string, string>() // (BD name -> Unity model name)
    {
        { "fl.hx", "fl.hip" },
        { "fl.hy", "fl.uleg" },
        { "fl.kn" , "fl.lleg" },
        { "fr.hx", "fr.hip" },
        { "fr.hy" , "fr.uleg" },
        { "fr.kn" , "fr.lleg" },
        { "hl.hx" , "hl.hip" },
        { "hl.hy" , "hl.uleg" },
        { "hl.kn" , "hl.lleg" },
        { "hr.hx" , "hr.hip" },
        { "hr.hy" , "hr.uleg" },
        { "hr.kn" , "hr.lleg" },
    };

    /*
    public static Axis GetJointRotationAxis(string jointUnityModelName)
    {
        if (jointUnityModelName.Contains("hip"))
        {
            return Axis.Z;
        }
        else if (jointUnityModelName.Contains("uleg"))
        {
            return Axis.X;
        }
        else if (jointUnityModelName.Contains("lleg"))
        {
            return Axis.X;
        }
        else
        {
            Debug.LogError("Joint " + jointUnityModelName + " not able to map to a rotation axis");
            return Axis.None;
        }
    }
    */

    // Converts position value (Spot's joint angle) to Euler Angle for the Unity model
    public static Vector3 JointPositionToEulerAngles(SpotJoint joint)
    {
        float jointPosition = joint.position;
        string jointModelName = jointNameMap[joint.name];
        float angle = jointPosition; // Radians
        angle = Mathf.Rad2Deg * angle;
        Vector3 rot = new Vector3(0f, 0f, 0f);
        /*
        switch (SpotJoint.GetJointRotationAxis(jointModelName))
        {
            case Axis.X:
                rot = Vector3.right * angle;
                break;
            case Axis.Z:
                rot = Vector3.forward * -angle;
                break;
            default:
                break;
        }
        */
        if (jointModelName.Contains("hip"))
        {
            rot = Vector3.forward * -angle;
        }
        else if (jointModelName.Contains("uleg"))
        {
            rot = Vector3.right * angle;
        }
        else if (jointModelName.Contains("lleg"))
        {
            rot = Vector3.right * angle;
        }
        return rot;
    }

    public static GameObject GetJointGameObject(string spotJointName)
    {
        /*
        string jointModelName = SpotJoint.jointNameMap[spotJointName];
        GameObject modelJoint = GameObject.Find(jointModelName);
        Assert.IsNotNull(modelJoint, "Joint '" + jointModelName + "' unable to be found");
        return modelJoint;
        */

        string jointModelName = SpotJoint.jointNameMap[spotJointName];
        foreach(GameObject model in GameObject.FindGameObjectsWithTag("spotJoint"))
        {
            if (model.name == jointModelName)
                return model;
        }

        return null;
    }
}
