using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GoToReceivePointer : MonoBehaviour
{
    private PointerManager pointerManager = null;

    // Start is called before the first frame update
    void Start()
    {
        pointerManager = FindObjectOfType<PointerManager>();
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void ReceivePoint(MixedRealityPointerEventData eventData)
    {
        var result = eventData.Pointer.Result;
        Vector3 point = result.Details.Point;

        bool goToResult = false;
        try
        {
            goToResult = GameObject.FindObjectOfType<VelocityManager>().GoToPointCommand(point.x, point.y, point.z);
            if (goToResult)
                ;
            else
                pointerManager.SendPointerFlashSignal(eventData.Handedness);

        }
        catch (Exception ex)
        {
            Debug.LogException(ex, this);
            pointerManager.SendPointerFlashSignal(eventData.Handedness);
        }
    }

    /*
    public void ReceivePoint(MixedRealityPointerEventData eventData)
    {
        var result = eventData.Pointer.Result;
        Vector3 point = result.Details.Point;

        bool goToResult = false;
        try
        {
            goToResult = GameObject.FindObjectOfType<VelocityManager>().GoToPointCommand(point.x, point.y, point.z);
            if (goToResult)
                PointerFlash(true);
            else
                PointerFlash(false);

        }
        catch (Exception ex)
        {
            Debug.LogException(ex, this);
            PointerFlash(false);
        }
    }

    private void PointerFlash(bool good)
    {
        if(pointerManager != null)
        {
            if (good == false)
                pointerManager.FlashRed(this, null);
            else
                ;
        }
    }
    */
}
