using Newtonsoft.Json.Bson;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROS2CameraManager : MonoBehaviour
{
    public GameObject cameraViewPrefab;

    //private string defaultCameraName = "frontright_fisheye_image";

    private void Awake()
    {
        
    }

    public void CreateCameraView()
    {
        Vector3 position = Camera.main.transform.position;
        position = position + (Camera.main.transform.forward * 0.5f);
        Quaternion rotation = Camera.main.transform.rotation;
        var cameraViewObject = Instantiate(cameraViewPrefab, position, rotation);
        //cameraViewObject.GetComponentInChildren<ROS2CameraSubscriber>().SetCamera(defaultCameraName);
    }
}
