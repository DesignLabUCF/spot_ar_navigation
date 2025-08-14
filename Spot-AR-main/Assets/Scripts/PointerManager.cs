using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static System.Net.WebRequestMethods;

public class PointerManager : MonoBehaviour
{
    //public Color normalColor;
    //public Color invalidSelectionColor;

    private SENSEableShellHandRayPointer leftPointer;
    private SENSEableShellHandRayPointer rightPointer;

    public VelocityManager velocityManager;
    private bool pointerActive = true;

    private IEnumerator handRay1Coroutine = null;
    private IEnumerator handRay2Coroutine = null;
    private int lerpIterations = 20;
    private float colorFadeTime = 1.0f;

    public bool useEndpoints = true;
    public Material endpointMaterialStopped = null;
    public Material endpointMaterialNotStopped = null;

    void Start()
    {
        //TurnRaysOff();

        velocityManager.controlTypeChanged += SetRays;
        velocityManager.movementStopStatusChanged += SetPointEndpointMaterials;
        //velocityManager.pointCommandIssueFailed += FlashRed;

        /*
        // For testing
        TurnRaysOff();
        Invoke("TurnRaysOn", 7.0f);
        */

        /*
        Debug.Log("Spatial Awareness Mask: " + LayerMask.GetMask("Spatial Awareness"));
        Debug.Log("Layer mask length: " + CoreServices.InputSystem.InputSystemProfile.PointerProfile.PointingRaycastLayerMasks.Length);
        Debug.Log(CoreServices.InputSystem.InputSystemProfile.PointerProfile.PointingRaycastLayerMasks[0].value);
        CoreServices.InputSystem.InputSystemProfile.PointerProfile.PointingRaycastLayerMasks[0].value = (1 << 31);
        Debug.Log(CoreServices.InputSystem.InputSystemProfile.PointerProfile.PointingRaycastLayerMasks[0].value);
        */
    }

    void Update()
    {
        if(useEndpoints && CoreServices.FocusProvider.GetPointers<SENSEableShellHandRayPointer>().Count() > 0)
        {
            UpdatePointer(Handedness.Right);
            UpdatePointer(Handedness.Left);
        }
    }

    private void OnEnable()
    {
        
    }

    private void UpdatePointer(Handedness hand)
    {
        var pointer = GetPointer(hand);
        Vector3 pointerBasePosition = pointer.gameObject.transform.position;

        Vector3 endPoint = Vector3.zero;
        PointerUtils.TryGetPointerEndpoint<SENSEableShellHandRayPointer>(hand, out endPoint);

        float rayMagnitude = (endPoint - pointerBasePosition).magnitude + 0.005f;
        //Debug.Log(rayMagnitude);
        //Debug.Log("T: " + pointer.DefaultPointerExtent);
        
        if(rayMagnitude < pointer.DefaultPointerExtent)
        {
            pointer.SetEndpoint(true, endPoint, Quaternion.identity);
        }
        else
        {
            pointer.SetEndpoint(false, Vector3.zero, Quaternion.identity);
        }
    }

    private void SetRays(object sender, string e)
    {
        // Will have to update this if more control methods are added. For now, they're a bool
        bool shouldBeVisible = e.ToUpper() == "TRUE";
        if(shouldBeVisible) 
        {
            TurnRaysOn();
        }
        else
        {
            TurnRaysOff();
        }
    }

    private void TurnRaysOff()
    {
        //PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOff, Handedness.Both);
        PointerUtils.SetPointerBehavior<SENSEableShellHandRayPointer>(PointerBehavior.AlwaysOff, Handedness.Both);
        pointerActive = false;
    }

    private void TurnRaysOn()
    {
        //PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOn, Handedness.Both);
        PointerUtils.SetPointerBehavior<SENSEableShellHandRayPointer>(PointerBehavior.AlwaysOn, Handedness.Both);
        pointerActive = true;
    }

    public void SendPointerFlashSignal(Handedness hand)
    {
        if(pointerActive == false)
        {
            return;
        }

        var pointer = GetPointer(hand);
        if(pointer != null)
        {
            pointer.FlashRed(this, null);
        }
        else
        {
            Debug.LogError("Pointer for hand '" + hand + "' not found.");
        }
    }

    private SENSEableShellHandRayPointer GetPointer(Handedness hand)
    {
        if (leftPointer == null || rightPointer == null)
        {
            var pointers = FindObjectsOfType<SENSEableShellHandRayPointer>(); //  Maybe use instead CoreServices.FocusProvider.GetPointers<SENSEableShellHandRayPointer>();
            foreach (var pointer in pointers)
            {
                if(pointer.gameObject.name.ToLower().Contains("left"))
                {
                    leftPointer = pointer;
                    continue;
                }
                else if(pointer.gameObject.name.ToLower().Contains("right"))
                {
                    rightPointer = pointer;
                    continue;
                }
            }
        }

        if(hand == Handedness.Left)
            return leftPointer;
        else
            return rightPointer;
    }

    private void SetPointEndpointMaterials(object sender, bool e)
    {
        if(e == true)
        {
            GetPointer(Handedness.Right).SetEndpointMaterial(endpointMaterialStopped);
            GetPointer(Handedness.Left).SetEndpointMaterial(endpointMaterialStopped);
        }
        else
        {
            GetPointer(Handedness.Right).SetEndpointMaterial(endpointMaterialNotStopped);
            GetPointer(Handedness.Left).SetEndpointMaterial(endpointMaterialNotStopped);
        }
    }

    /*
    public void FlashRed(object sender, EventArgs e)
    {
        Debug.Log("TODO: Make pointer flash red if visible...");

        CancelFlashes();

        foreach (var ray in FindObjectsOfType<ShellHandRayPointer>())
        {
            Debug.Log(ray.gameObject.name);
            var mixedRealityLineRenderer = ray.GetComponent<MixedRealityLineRenderer>();
            if(handRay1Coroutine == null)
            {
                handRay1Coroutine = LaunchFlashTimer(mixedRealityLineRenderer);
                StartCoroutine(handRay1Coroutine);
            }
            else if(handRay2Coroutine == null)
            {
                handRay2Coroutine = LaunchFlashTimer(mixedRealityLineRenderer);
                StartCoroutine(handRay2Coroutine);
            }
        }
    }

    private IEnumerator LaunchFlashTimer(MixedRealityLineRenderer lineRenderer)
    {
        for(int i = 0; i < lerpIterations; i++)
        {
            //lineRenderer.LineColor = GradientUtil.BlendTwoGradients(invalidSelectionGradient, normalGradient, i / lerpIterations, 64);
            int colorKeyLength = lineRenderer.LineColor.colorKeys.Length;
            var colorKey = new GradientColorKey[2];
            for (int j = 0; j < colorKeyLength; j++)
            {

                colorKey[j].time = lineRenderer.LineColor.colorKeys[j].time;
                colorKey[j].color = Color.Lerp(invalidSelectionColor, normalColor, (float)i / (float)lerpIterations);
            }
            lineRenderer.LineColor.SetKeys(colorKey, lineRenderer.LineColor.alphaKeys);

            yield return new WaitForSeconds(colorFadeTime / (float)lerpIterations);
        }
        for (int j = 0; j < lineRenderer.LineColor.colorKeys.Length; j++)
        {
            lineRenderer.LineColor.colorKeys[j].color = normalColor;
        }
    }

    private void CancelFlashes()
    {
        foreach (var ray in FindObjectsOfType<ShellHandRayPointer>())
        {
            var mixedRealityLineRenderer = ray.GetComponent<MixedRealityLineRenderer>();
            for (int j = 0; j < mixedRealityLineRenderer.LineColor.colorKeys.Length; j++)
            {
                mixedRealityLineRenderer.LineColor.colorKeys[j].color = normalColor;
            }
        }

        if (handRay1Coroutine != null)
        {
            StopCoroutine(handRay1Coroutine);
            handRay1Coroutine = null;
        }
        if (handRay2Coroutine != null)
        {
            StopCoroutine(handRay2Coroutine);
            handRay2Coroutine = null;
        }
    }
    */
}