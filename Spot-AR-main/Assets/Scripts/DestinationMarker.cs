using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DestinationMarker : MonoBehaviour
{
    public bool shouldRenderLine = true;

    public VelocityManager velocityManager;
    //private List<GameObject> modelParts = new List<GameObject>();
    public GameObject arrowModel;
    public LineRenderer lineRenderer;

    private IEnumerator displayCoroutine = null;

    public float displayDuration = 20.0f; // Seconds

    private void Awake()
    {
        /*
        for(int i = 0; i < transform.childCount; i++)
        {
            modelParts.Add(transform.GetChild(i).gameObject);
        }
        */
        SetVisibility(false);

        velocityManager.spotDestinationSent += ReceiveDestination;
        velocityManager.spotDestinationCancel += CancelDestination;
        velocityManager.movementStopStatusChanged += CancelDestination;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void SetVisibility(bool visibility)
    {
        /*
        foreach (GameObject part in modelParts)
        {
            part.SetActive(visibility);
        }
        */
        arrowModel.SetActive(visibility);
        lineRenderer.enabled = visibility && shouldRenderLine;
        if (visibility)
        {

        }
        else
        {
            
        }
    }

    private void ReceiveDestination(object sender, Vector3[] e)
    {
        if (displayCoroutine != null)
        {
            StopCoroutine(displayCoroutine);
            displayCoroutine = null;
        }
        displayCoroutine = DisplayTemporarily(e);
        StartCoroutine(displayCoroutine);
    }

    private IEnumerator DisplayTemporarily(Vector3[] points)
    {
        SetVisibility(true);

        arrowModel.transform.position = points[1];
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, points[0]);
        lineRenderer.SetPosition(1, points[1]);

        for (int i = 0; i < (int)displayDuration; i++)
        {
            yield return new WaitForSeconds(1.0f);
        }

        SetVisibility(false);
    }

    private void CancelDestination(object sender, EventArgs e)
    {
        if (displayCoroutine != null)
        {
            StopCoroutine(displayCoroutine);
            displayCoroutine = null;
        }
        SetVisibility(false);
    }

    private void CancelDestination(object sender, bool e)
    {
        CancelDestination(this, null);
    }
}
