using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StopHandManager : MonoBehaviour
{
    public VelocityManager velocityManager;
    public MeshRenderer indicatorMeshRenderer;

    [Header("Indicator Materials")]
    public Material normalMaterial = null;
    public Material stoppedMaterial = null;

    private void Awake()
    {

    }

    void Start()
    {
        velocityManager.movementStopStatusChanged += UpdateStopMaterial;
    }

    void Update()
    {
        
    }

    public void SendStopSignal()
    {
        velocityManager.ReceiveStopInput();
    }

    private void UpdateStopMaterial(object sender, bool e)
    {
        if(e == true)
        {
            indicatorMeshRenderer.material = stoppedMaterial;
        }
        else
        {
            indicatorMeshRenderer.material = normalMaterial;
        }
    }
}
