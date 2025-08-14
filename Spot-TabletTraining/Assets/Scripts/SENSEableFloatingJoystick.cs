using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.EventSystems;

public class SENSEableFloatingJoystick : Joystick
{
    //[SerializeField] private bool isLeft = false;
    //public float testVal = 1.0f;

    protected override void Start()
    {
        base.Start();
        background.gameObject.SetActive(false);
    }

    public override void OnPointerDown(PointerEventData eventData)
    {
        UnityEngine.Debug.Log(eventData.position);
        UnityEngine.Debug.Log("Joystick is Left: " + JoystickIsLeft());
        //UnityEngine.Debug.Log("Position.x: " + eventData.position.x);
        if (JoystickIsLeft() == true && eventData.position.x <= (Screen.width / 2.0f))
        {
            //UnityEngine.Debug.Log("Left");
        }
        else if(JoystickIsLeft() == false && eventData.position.x > (Screen.width / 2.0f))
        {
            //UnityEngine.Debug.Log("Right");
        }
        else
        {
            return;
        }

        background.anchoredPosition = ScreenPointToAnchoredPosition(eventData.position);
        background.gameObject.SetActive(true);
        base.OnPointerDown(eventData);
    }

    public override void OnPointerUp(PointerEventData eventData)
    {
        background.gameObject.SetActive(false);
        base.OnPointerUp(eventData);
    }

    private bool JoystickIsLeft()
    {
        return transform.name.ToUpper().Contains("LEFT");
    }
}