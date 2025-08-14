using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;

public class GestureManager : MonoBehaviour, IMixedRealityGestureHandler<Vector3>
{
    [SerializeField]
    private MixedRealityInputAction testAction = MixedRealityInputAction.None;

    private float fingerUpThreshold = 0.3f;
    private float fingerDownThreshold = 0.7f;

    private float thumbCurl = 0f;
    private float indexCurl = 0f;
    private float middleCurl = 0f;
    private float ringCurl = 0f;
    private float pinkyCurl = 0f;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        thumbCurl = HandPoseUtils.ThumbFingerCurl(Handedness.Right);
        indexCurl = HandPoseUtils.IndexFingerCurl(Handedness.Right);
        middleCurl = HandPoseUtils.MiddleFingerCurl(Handedness.Right);
        ringCurl = HandPoseUtils.RingFingerCurl(Handedness.Right);
        pinkyCurl = HandPoseUtils.PinkyFingerCurl(Handedness.Right);
    }

    private void OnEnable()
    {
        CoreServices.InputSystem?.RegisterHandler<IMixedRealityGestureHandler>(this); // Necessary for global recognition, and not just on a collider
    }

    private void OnDisable()
    {
        CoreServices.InputSystem?.UnregisterHandler<IMixedRealityGestureHandler>(this); // Necessary for global recognition, and not just on a collider
    }

    public void OnGestureUpdated(InputEventData<Vector3> eventData)
    {
        throw new System.NotImplementedException();
    }

    public void OnGestureCompleted(InputEventData<Vector3> eventData)
    {
        throw new System.NotImplementedException();
    }

    public void OnGestureStarted(InputEventData eventData)
    {
        //Debug.Log("Gesture started: " + eventData.MixedRealityInputAction.Description);
    }

    public void OnGestureUpdated(InputEventData eventData)
    {
        //Debug.Log("Gesture updated: " + eventData.MixedRealityInputAction.Description);
    }

    public void OnGestureCompleted(InputEventData eventData)
    {
        //Debug.Log("Gesture completed: " + eventData.MixedRealityInputAction.Description);
    }

    public void OnGestureCanceled(InputEventData eventData)
    {
        ;
    }
}
