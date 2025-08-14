using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public SENSEableFloatingJoystick leftJoystick;
    public SENSEableFloatingJoystick rightJoystick;
    private Rigidbody rb;

    private bool controlEnabled = true;
    public float forwardSpeed = 1.0f;
    public float strafeSpeed = 1.0f;
    public float rotateSpeed = 1.0f;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if(controlEnabled == true)
        {
            if (leftJoystick.Vertical >= 0.1f || leftJoystick.Vertical <= -0.1f)
            {
                //UnityEngine.Debug.Log("Forward/back");
                Vector3 forwardVelocity = transform.forward * forwardSpeed * leftJoystick.Vertical * Time.fixedDeltaTime;
                rb.velocity = rb.velocity + forwardVelocity;
            }
            if (leftJoystick.Horizontal >= 0.1f || leftJoystick.Horizontal <= -0.1f)
            {
                Vector3 rightVelocity = transform.right * strafeSpeed * leftJoystick.Horizontal * Time.fixedDeltaTime;
                rb.velocity = rb.velocity + rightVelocity;
            }
            if (rightJoystick.Horizontal >= 0.1f || rightJoystick.Horizontal <= -0.1f)
            {
                Vector3 rotateVelocity = Vector3.up * rotateSpeed * rightJoystick.Horizontal * Time.fixedDeltaTime;
                rb.angularVelocity = rb.angularVelocity + rotateVelocity;
            }
        }
    }

    public void TakeAwayControl()
    {
        controlEnabled = false;
        leftJoystick.gameObject.SetActive(false);
        rightJoystick.gameObject.SetActive(false);
    }
}
