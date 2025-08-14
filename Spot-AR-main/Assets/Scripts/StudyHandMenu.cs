using Microsoft.MixedReality.Toolkit.UI;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Tracing;
using TMPro;
using UnityEngine;
using UnityEngine.UIElements;

public class StudyHandMenu : MonoBehaviour
{
    public enum Tool
    {
        Joystick = 0,
        ROS = 1,
        Commands = 2
    }

    private Interactable[] toggles;

    [Header("Manual Joystick Controls")]
    public GameObject joystickBaseGameObject = null;
    public Interactable joystickToggle = null;
    [Header("ROS2 Settings")]
    public GameObject rosBaseGameObject = null;
    public Interactable rosToggle = null;
    [Header("Commands List")]
    public GameObject commandsBaseGameObject = null;
    public Interactable commandsToggle = null;
    [Header("Invert Button")]
    public TextMeshPro textInvertHeadingLabel;
    public ButtonConfigHelper invertButtonConfigHelper;
    public Texture forwardTexture;
    public Texture backwardTexture;
    [Header("Stop Button")]
    public UnityEngine.UI.Image stoppedDisplay;
    [Header("Scene References")]
    public VelocityManager velocityManager;
    public ROS2ConnectionDisplay ros2ConnectionDisplay;
    [Header("Menu Alert References")]
    public UnityEngine.UI.Image alertDisplay;
    private IEnumerator alertCoroutine = null;

    private void Awake()
    {
        Interactable[] toggles = { joystickToggle, rosToggle, commandsToggle };
        this.toggles = toggles;
    }

    // Start is called before the first frame update
    void Start()
    {
        velocityManager.movementStopStatusChanged += UpdateStopButtonDisplay;
        velocityManager.invertHeadingChanged += UpdateInvertHeadingLabel;
        velocityManager.pointCommandIssueFailed += MakeMenuFlashRed;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnEnable()
    {
        UpdateStopButtonDisplay(this, velocityManager.MovementIsStopped);
        UpdateInvertHeadingLabel(this, velocityManager.Invert_heading ? "True" : "False");
    }

    public void SetCurrentActiveTool(int toolReferenceIndex)
    {  
        // Disable all tools
        if(joystickBaseGameObject != null)
            joystickBaseGameObject.SetActive(false);
        if (rosBaseGameObject != null)
            rosBaseGameObject.SetActive(false);
        if (commandsBaseGameObject != null)
            commandsBaseGameObject.SetActive(false);

        Tool tool = (Tool)toolReferenceIndex;
        Interactable toggle = null;
        switch (tool)
        {
            case Tool.Joystick:
                toggle = joystickToggle;
                joystickBaseGameObject.SetActive(toggle.IsToggled);
                velocityManager.SetControlType(!toggle.IsToggled); // Switch to manual velocity controls
                break;
            case Tool.ROS:
                toggle = rosToggle;
                rosBaseGameObject.SetActive(toggle.IsToggled);
                //ros2ConnectionDisplay.CancelAlert();
                break;
            case Tool.Commands:
                toggle = commandsToggle;
                commandsBaseGameObject.SetActive(toggle.IsToggled);
                break;
            default:
                ;
                break;
        }

        DeactivateAllOtherToggles(toggle);
    }

    private void DeactivateAllOtherToggles(Interactable toggleToIgnore)
    {
        foreach(Interactable toggle in toggles)
        {
            //toggle.SetState(InteractableStates.InteractableStateEnum.Toggled, false);
            if (toggle == toggleToIgnore)
                continue;

            if (toggle != null)
                toggle.SetToggled(false);
        }
    }

    private void UpdateStopButtonDisplay(object sender, bool e)
    {
        if (e == true)
        {
            stoppedDisplay.color = new Color(stoppedDisplay.color.r, stoppedDisplay.color.g, stoppedDisplay.color.b, 1.0f);
        }
        else
        {
            stoppedDisplay.color = new Color(stoppedDisplay.color.r, stoppedDisplay.color.g, stoppedDisplay.color.b, 0f);
        }
    }

    public void UpdateInvertHeadingLabel(object sender, string e)
    {
        if (e == "True")
        {
            textInvertHeadingLabel.text = "Walking\nBackward";
            invertButtonConfigHelper.SetQuadIcon(backwardTexture);
        }
        else
        {
            textInvertHeadingLabel.text = "Walking\nForward";
            invertButtonConfigHelper.SetQuadIcon(forwardTexture);
        }
    }

#region Menu Flash
    private void MakeMenuFlashRed(object sender, EventArgs e)
    {
        CancelAlert();
        if (this.isActiveAndEnabled)
        {
            alertDisplay.color = new Color(1.0f, 0f, 0f, 1.0f);
            alertCoroutine = FadeAlertToTransparent();
            StartCoroutine(alertCoroutine);
        }
    }

    public void CancelAlert()
    {
        alertDisplay.color = new Color(1.0f, 0f, 0f, 0.0f);
        if (alertCoroutine != null)
        {
            StopCoroutine(alertCoroutine);
            alertCoroutine = null;
        }
    }

    private IEnumerator FadeAlertToTransparent()
    {
        int max = 20;
        for (int i = max; i > 0; i--)
        {
            alertDisplay.color = new Color(1.0f, 0f, 0f, i / (float)max);
            yield return new WaitForSeconds(0.05f);
        }
        alertDisplay.color = new Color(1.0f, 0f, 0f, 0f);
    }
#endregion
}
