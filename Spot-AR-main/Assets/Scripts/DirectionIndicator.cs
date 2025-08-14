using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using TMPro;
using UnityEngine;

public class DirectionIndicator : MonoBehaviour
{
    public VelocityManager velocityManager;
    public GameObject arrowBaseGameobject;
    //public GameObject arrowBase;
    //public GameObject arrowHead;
    public GameObject arrowModel;
    public TextMeshPro[] stoppedTexts;
    public TextMeshPro controlTypeText;
    public TextMeshPro invertHeadingText;

    [Header("Indicator Materials")]
    public Material normalMaterial = null;
    public Material stoppedMaterial = null;

    private void Awake()
    {

    }

    private void Start()
    {
        velocityManager.invertHeadingChanged += UpdateArrowDirection;
        velocityManager.movementStopStatusChanged += UpdateArrowColor;
        velocityManager.controlTypeChanged += UpdateControlTypeText;
    }

    private void Update()
    {
        MakeTextFaceUser();
    }

    private void OnEnable()
    {
        UpdateArrowDirection(this, velocityManager.Invert_heading ? "True" : "False");
        UpdateArrowColor(this, velocityManager.MovementIsStopped);
        UpdateControlTypeText(this, velocityManager.UsePointControls ? "True" : "False");
    }

    private void SetVisibility(bool visibility)
    {
        //arrowBase.SetActive(visibility);
        //arrowHead.SetActive(visibility);
    }

    private void UpdateArrowDirection(object sender, string e)
    {
        if (e == "True")
        {
            arrowBaseGameobject.transform.localRotation = Quaternion.Euler(0f, 180.0f, 0f);
            invertHeadingText.text = "Walking Backwards";
        }
        else
        {
            arrowBaseGameobject.transform.localRotation = Quaternion.Euler(0f, 0, 0f);
            invertHeadingText.text = "Walking Forward";
        }
    }

    private void UpdateArrowColor(object sender, bool e)
    {
        if(e)
        {
            SetArrowColor(stoppedMaterial);
            SetStoppedText(true);
        }
        else
        {
            SetArrowColor(normalMaterial);
            SetStoppedText(false);
        }
    }

    private void SetArrowColor(Material mat)
    {
        //arrowBase.GetComponent<MeshRenderer>().material = mat;
        //arrowHead.GetComponent<MeshRenderer>().material = mat;
        arrowModel.GetComponent<MeshRenderer>().material = mat;
    }

    private void SetStoppedText(bool shouldShow)
    {
        foreach (TextMeshPro stoppedText in stoppedTexts)
        {
            stoppedText.gameObject.SetActive(shouldShow);
        }
    }

    private void UpdateControlTypeText(object sender, string e)
    {
        if(e == "True") 
        {
            //controlTypeText.text = "Directed Controls";
            controlTypeText.text = "";
        }
        else
        {
            //controlTypeText.text = "Joystick Controls";
            controlTypeText.text = "[POINTING DISABLED]";
        }
    }

    private void MakeTextFaceUser()
    {
        controlTypeText.transform.LookAt(Camera.main.transform);
        controlTypeText.transform.localEulerAngles = Vector3.Scale(controlTypeText.transform.localEulerAngles, new Vector3(0f, 1.0f, 0f)); // Use only pivot around y angle

        invertHeadingText.transform.LookAt(Camera.main.transform);
        invertHeadingText.transform.localEulerAngles = Vector3.Scale(invertHeadingText.transform.localEulerAngles, new Vector3(0f, 1.0f, 0f)); // Use only pivot around y angle
    }
}
