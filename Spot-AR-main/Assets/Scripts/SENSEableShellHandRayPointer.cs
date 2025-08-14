using Microsoft.MixedReality.OpenXR;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Physics;
using Microsoft.MixedReality.Toolkit.Utilities;
using System;
using System.Collections;
using UnityEditor;
using UnityEngine;

public class SENSEableShellHandRayPointer : ShellHandRayPointer
{
    [Header("SENSEable - Endpoint")]
    public GameObject endPointObject;

    [Header("SENSEable - Color Overrides")]
    public Color normalColor = Color.green;
    public Color invalidSelectionColor = Color.red;

    //private VelocityManager velocityManager;
    private MixedRealityLineRenderer lineRenderer;

    private IEnumerator handRayCoroutine = null;
    private int lerpIterations = 20;
    private float colorFadeTime = 3.0f;

    protected override void Start()
    {
        base.Start();

        //velocityManager = FindObjectOfType<VelocityManager>();
        //velocityManager.pointCommandIssueFailed += FlashRed;

        SetEndpoint(false, Vector3.zero, Quaternion.identity);
    }

    protected override void OnEnable()
    {
        base.OnEnable();

        CancelFlash();
    }

    /*
    public override void OnInputDown(InputEventData eventData)
    {
        base.OnInputDown(eventData);

        CancelFlash();
    }
    */

    /*
    protected void Update()
    {
        LayerMask[] layerMasks = { LayerMask.GetMask("Spatial Awareness") };

        GameObject hitObject = null;
        Vector3 hitPoint = Vector3.zero;
        float hitDistance = -1.0f;
        MixedRealityRaycastHit hitInfo;
        
        OnSceneQuery(layerMasks, false, out hitObject, out hitPoint, out hitDistance);
        Debug.Log(hitPoint);

        RayStep ray = new RayStep(raycastOrigin.position, raycastOrigin.rotation.eulerAngles);
        CoreServices.InputSystem.RaycastProvider.Raycast(ray, layerMasks, false, out hitInfo);
        Debug.Log(hitInfo);
    }
    */

    public void SetEndpoint(bool visible, Vector3 position, Quaternion rotation)
    {
        endPointObject.SetActive(visible);
        endPointObject.GetComponent<MeshRenderer>().enabled = visible;
        endPointObject.transform.position = position;
    }

    public void SetEndpointMaterial(Material material)
    {
        endPointObject.GetComponent<MeshRenderer>().material = material;
    }

    private MixedRealityLineRenderer LineRenderer()
    {
        if (lineRenderer != null)
            return lineRenderer;
        
        lineRenderer = GetComponent<MixedRealityLineRenderer>();
        return lineRenderer;
    }

    public void FlashRed(object sender, EventArgs e)
    {
        CancelFlash();

        if(isActiveAndEnabled)
        {
            handRayCoroutine = LaunchFlashTimer();
            StartCoroutine(handRayCoroutine);
        }

        /*
        if (handRayCoroutine == null)
        {
            handRayCoroutine = LaunchFlashTimer();
            StartCoroutine(handRayCoroutine);
        }
        */
    }

    private IEnumerator LaunchFlashTimer()
    {
        for (int i = 0; i < lerpIterations; i++)
        {
            Color desiredColor = Color.Lerp(invalidSelectionColor, normalColor, (float)i / (float)lerpIterations);
            //Debug.Log("Setting " + this.Handedness + " to: " + desiredColor);
            SetColorKey(desiredColor);

            yield return new WaitForSeconds(colorFadeTime / (float)lerpIterations);
        }
        SetColorKey(normalColor);
    }

    private void CancelFlash()
    {
        if (handRayCoroutine != null)
        {
            StopCoroutine(handRayCoroutine);
            handRayCoroutine = null;
        }

        SetColorKey(normalColor);
    }

    private void SetColorKey(Color color)
    {
        if(LineRenderer() == null)
        {
            Debug.Log("LineRenderer not found");
            return;
        }

        int colorKeyLength = LineRenderer().LineColor.colorKeys.Length;

        var colorKey = new GradientColorKey[colorKeyLength];
        for (int j = 0; j < colorKeyLength; j++)
        {
            colorKey[j].time = LineRenderer().LineColor.colorKeys[j].time;
            colorKey[j].color = color;
        }
        //LineRenderer().LineColor.SetKeys(colorKey, LineRenderer().LineColor.alphaKeys);
        LineColorInvalid.SetKeys(colorKey, LineRenderer().LineColor.alphaKeys);
        LineColorValid.SetKeys(colorKey, LineRenderer().LineColor.alphaKeys);
        LineColorNoTarget.SetKeys(colorKey, LineRenderer().LineColor.alphaKeys);
    }
}
