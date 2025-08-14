using UnityEngine;

public class VirtualKeyboardManager : MonoBehaviour
{
    public ROS2Manager ROS2Manager;
    public ParticipantLogger participantLogger;

    public TouchScreenKeyboard hl2Keyboard;
    //private bool lastActiveKeyboardInputWasIP = true; // A quick dirty way to do this since we only have two keyboard input spots

    private AvailableKeyboards lastKeyboard = AvailableKeyboards.IP;
    private enum AvailableKeyboards
    {
        IP,
        Port,
        ParticipantID
    }

    private void Start()
    {
        TouchScreenKeyboard.hideInput = false;
    }

    private void Update()
    {
        // Parse keyboard
        if (hl2Keyboard != null)
        {
            /*
            if (lastActiveKeyboardInputWasIP)
            {
                ROS2Manager.SetIP(hl2Keyboard.text.ToString());
            }
            else
            {
                ROS2Manager.SetPort(int.Parse(hl2Keyboard.text));
            }
            */
            switch(lastKeyboard)
            {
                case AvailableKeyboards.IP:
                    ROS2Manager.SetIP(hl2Keyboard.text.ToString());
                    break;
                case AvailableKeyboards.Port:
                    ROS2Manager.SetPort(int.Parse(hl2Keyboard.text));
                    break;
                case AvailableKeyboards.ParticipantID:
                    participantLogger.SetParticipantID(hl2Keyboard.text.ToString());
                    break;
                default:
                    break;

            }
        }
    }

    public void LaunchKeyboardIPAddress()
    {
        //ipKeyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.NumberPad, false, false, false, false, "0.0.0.0", 0);
        lastKeyboard = AvailableKeyboards.IP;
        hl2Keyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.NumberPad, false, false, false, false, "0.0.0.0", 0);
    }

    public void LaunchKeyboardPort()
    {
        //portKeyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.PhonePad, false, false, false, false, "0.0.0.0", 0);
        lastKeyboard = AvailableKeyboards.Port;
        hl2Keyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.PhonePad, false, false, false, false, "0.0.0.0", 0);
    }

    public void LaunchKeyboardParticipantID()
    {
        lastKeyboard = AvailableKeyboards.ParticipantID;
        hl2Keyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.Default, false, false, false, false, "UNSET", 0);
    }
}
