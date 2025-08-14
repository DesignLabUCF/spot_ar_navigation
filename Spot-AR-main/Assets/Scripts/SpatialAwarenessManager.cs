using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.SpatialAwareness;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Assertions.Must;

public class SpatialAwarenessManager : MonoBehaviour
{
    private void Start()
    {
        /*
        // https://learn.microsoft.com/en-us/windows/mixed-reality/mrtk-unity/mrtk2/features/spatial-awareness/usage-guide?view=mrtkunity-2022-05
        var spatialAwarenessService = CoreServices.SpatialAwarenessSystem;
        var dataProviderAccess = spatialAwarenessService as IMixedRealityDataProviderAccess;
        var meshObserver = dataProviderAccess.GetDataProvider<IMixedRealitySpatialAwarenessMeshObserver>();
        */
    }

    private void Update()
    {
        var meshObserver = CoreServices.GetSpatialAwarenessSystemDataProvider<IMixedRealitySpatialAwarenessMeshObserver>();

        if (meshObserver != null)
        {
            foreach (SpatialAwarenessMeshObject meshObject in meshObserver.Meshes.Values)
            {
                GameObject obj = meshObject.GameObject;
                Mesh mesh = meshObject.Filter.mesh;
                if(obj != null && obj.GetComponent<PointerHandler>() == null)
                {
                    var pointerHandler = obj.AddComponent<PointerHandler>();
                    var receiver = obj.AddComponent<GoToReceivePointer>();
                    pointerHandler.OnPointerClicked.AddListener(receiver.ReceivePoint);
                }
            }
        }
    }
}
