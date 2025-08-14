using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

public class Arrow : MonoBehaviour
{
    public float speed = 2.0f;
    public float height = 1.0f;

    private float baseHeight = 1.0f;

    void Start()
    {
        baseHeight = transform.localPosition.y;
    }

    private void Update()
    {
        // Add Floating effect
        //https://forum.unity.com/threads/how-to-make-an-object-move-up-and-down-on-a-loop.380159/
        Vector3 pos = transform.localPosition;
        //calculate what the new Y position will be
        float newY = Mathf.Sin(Time.time * speed);
        //set the object's Y to the new calculated Y
        transform.localPosition = new Vector3(pos.x, newY, pos.z) * height + new Vector3(0f, baseHeight, 0f);

        // Rotate to face player
        transform.LookAt(Camera.main.transform);
        transform.localEulerAngles = Vector3.Scale(transform.localEulerAngles, new Vector3(0f, 1.0f, 0f)); // Use only pivot around y angle
        transform.localEulerAngles = transform.localEulerAngles + new Vector3(180.0f, 0f, 0f); // Use only pivot around y angle
    }
}
