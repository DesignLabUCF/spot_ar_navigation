using Microsoft.MixedReality.Toolkit;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Tracing;
using System.Linq;
using System.Reflection.Emit;
using TMPro;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.UI;

public class PlaybackManager : MonoBehaviour
{
    public string data_path_no_extension = "";
    public GameObject spotOffsetParentObject;
    public GameObject spotModelBase;
    public Slider playbackSlider;
    public Button playbackButton;
    public TextMeshProUGUI durationText;
    public TextMeshProUGUI timestampText;
    public bool drawPathLine = false;
    public LineRenderer spotLineRenderer;
    public GameObject aprilTagMarkerData; // Tag position from Spot's data
    public GameObject aprilTagMarkerEnvironment; // Tag's pre-position position in the environment
    public GameObject dataTransform;

    private bool playbackActive = false;
    private IEnumerator playbackCoroutine = null;
    private readonly int UPDATE_FPS = 20;

    public List<SpotTransform> spotTransforms;
    /*
    Dictionary<string, string> jointNameMap = new Dictionary<string, string>() // (BD name -> Unity model name)
    {
        { "fl.hx", "fl.hip" },
        { "fl.hy", "fl.uleg" },
        { "fl.kn" , "fl.lleg" },
        { "fr.hx", "fr.hip" },
        { "fr.hy" , "fr.uleg" },
        { "fr.kn" , "fr.lleg" },
        { "hl.hx" , "hl.hip" },
        { "hl.hy" , "hl.uleg" },
        { "hl.kn" , "hl.lleg" },
        { "hr.hx" , "hr.hip" },
        { "hr.hy" , "hr.uleg" },
        { "hr.kn" , "hr.lleg" },
    };
    */

    private void Awake()
    {

    }

    private void Start()
    {
        GetSpotTransformData(data_path_no_extension);
        UpdateSpotModel();
    }

    private void Update()
    {

    }

    public void GetSpotTransformData(string resource_path_no_extension)
    {
        // Init and reset
        if (spotTransforms == null)
            spotTransforms = new List<SpotTransform>();
        spotTransforms.Clear();

        Func<string, float> ParsePotentialBlank = x => float.TryParse(x, out float y) == true ? float.Parse(x) : float.NaN; // Read in cell, if blank return NaN, else return it's value

        // Read in file
        var data = Resources.Load<TextAsset>(resource_path_no_extension);
        var rows = data.text.Split("\n");
        var headers = rows[0].Split(",");
        for (int i = 1; i < rows.Length - 1; i++)
        {
            var row = rows[i];
            var cells = row.Split(",");

            /*
             * Must convert from right hand Z-up (Boston Dynamics coords) to left hand Y-up (Unity coords)
             * Left hand => right hand - https://www.codeproject.com/Tips/1240454/How-to-Convert-Right-Handed-to-Left-Handed-Coordin
             * TODO
             */

            // Timestamp TODO
            string timestampStr = cells[1];
            string[] date = timestampStr.Split("_")[0].Split("-");
            int month = int.Parse(date[0]);
            int day = int.Parse(date[1]);
            int year = int.Parse(date[2]);
            string[] time = timestampStr.Split("_")[1].Split("-");
            int hour = int.Parse(time[0]);
            int minute = int.Parse(time[1]);
            int second = int.Parse(time[2]);
            int millisecond = int.Parse(time[3]);
            DateTime timestamp = new DateTime(year, month, day, hour, minute, second, millisecond);

            // Body data
            float posX = float.Parse(cells[2]);
            float posY = float.Parse(cells[3]);
            float posZ = float.Parse(cells[4]);
            Vector3 pos = new Vector3(posX, posZ, posY);

            float rotX = float.Parse(cells[5]);
            float rotY = float.Parse(cells[6]);
            float rotZ = float.Parse(cells[7]);
            float rotW = float.Parse(cells[8]);
            Quaternion rot = new Quaternion(-rotX, -rotZ, -rotY, rotW);
            rot = rot * Quaternion.Euler(0, 90.0f, 0); // Rotate quaternion additional 90 degrees around Y axis

            float velLinearX = float.Parse(cells[9]);
            float velLinearY = float.Parse(cells[10]);
            float velLinearZ = float.Parse(cells[11]);
            Vector3 linearVelocity = new Vector3(velLinearX, velLinearZ, velLinearY);

            float velAngularX = float.Parse(cells[12]);
            float velAngularY = float.Parse(cells[13]);
            float velAngularZ = float.Parse(cells[14]);
            Vector3 angularVelocity = new Vector3(velAngularX, velAngularZ, velAngularY);

            /*
            float aprilPosX = float.NaN;
            float aprilPosY = float.NaN;
            float aprilPosZ = float.NaN;
            float.TryParse(cells[15], out aprilPosX); // Cells can be empty if no april tag has yet been found
            float.TryParse(cells[16], out aprilPosY);
            float.TryParse(cells[17], out aprilPosZ);
            */
            float aprilPosX = ParsePotentialBlank(cells[15]);
            float aprilPosY = ParsePotentialBlank(cells[16]);
            float aprilPosZ = ParsePotentialBlank(cells[17]);
            Vector3? aprilPos = null;
            if(float.IsNaN(aprilPosX) == false) // Valid april tag detected
            {
                aprilPos = new Vector3(aprilPosX, aprilPosZ, aprilPosY);
            }

            /*
            float aprilRotX = float.NaN;
            float aprilRotY = float.NaN;
            float aprilRotZ = float.NaN;
            float aprilRotW = float.NaN;
            float.TryParse(cells[18], out aprilRotX);
            float.TryParse(cells[19], out aprilRotY);
            float.TryParse(cells[20], out aprilRotZ);
            float.TryParse(cells[21], out aprilRotW);
            */
            float aprilRotX = ParsePotentialBlank(cells[18]);
            float aprilRotY = ParsePotentialBlank(cells[19]);
            float aprilRotZ = ParsePotentialBlank(cells[20]);
            float aprilRotW = ParsePotentialBlank(cells[21]);
            Quaternion? aprilRot = null;
            if(float.IsNaN(aprilRotX) == false)
            {
                aprilRot = new Quaternion(-aprilRotX, -aprilRotZ, -aprilRotY, aprilRotW);
                aprilRot = aprilRot * Quaternion.Euler(0, 90.0f, 0); // Rotate quaternion additional 90 degrees around Y axis
            }

            // Joints
            List<SpotJoint> joints = new List<SpotJoint>();
            for (int j = 22; j < cells.Length; j += 4)
            {
                // Derive joint name from headers
                string jointName = headers[j].Split("_")[0];
                // Joint dara
                float jointPos = float.Parse(cells[j]); // Negative????
                float jointVelocity = float.Parse(cells[j + 1]);
                float jointAcceleration = float.Parse(cells[j + 2]);
                float jointLoad = float.Parse(cells[j + 3]);

                joints.Add(new SpotJoint(jointName, jointPos, jointVelocity, jointAcceleration, jointLoad));
            }
            // Append to mega-list
            spotTransforms.Add(new SpotTransform(timestamp, pos, rot, linearVelocity, angularVelocity, aprilPos, aprilRot, joints));
        }

        SetSliderLimits();
    }

    private void SetSliderLimits()
    {
        playbackSlider.minValue = 0;
        playbackSlider.maxValue = spotTransforms.Count - 1;
        playbackSlider.value = 0;
    }

    public void UpdateSpotModel()
    {
        // Read slider to determine what index in the transform list to use
        int desiredIndex = (int)playbackSlider.value;
        //print("Setting to " + desiredIndex.ToString());
        SpotTransform spotTransform = spotTransforms[desiredIndex];

        // Update slider timestamps
        durationText.text = (spotTransform.timestamp - spotTransforms[0].timestamp).TotalMilliseconds.ToString() + " ms";
        timestampText.text = spotTransform.timestamp.ToString("MM/dd/yyyy HH:mm:ss.fff");

        // April tag transform
        if (spotTransform.aprilPosition != null) // April tag is valid
        {
            // Set initial position and rotation of spot data for marker. Now we should have spot and the marker read in
            aprilTagMarkerData.SetActive(true);
            aprilTagMarkerData.transform.localPosition = (Vector3)spotTransform.aprilPosition;
            aprilTagMarkerData.transform.localRotation = (Quaternion)spotTransform.aprilRotation;


            Vector3 aprilTranslationPosition = aprilTagMarkerEnvironment.transform.position - (Vector3)spotTransform.aprilPosition;
            dataTransform.transform.position = aprilTranslationPosition;

            //dataTransform.transform.rotation = (Quaternion)spotTransform.aprilRotation * Quaternion.Inverse(aprilTagMarkerEnvironment.transform.rotation);
            dataTransform.transform.rotation = aprilTagMarkerEnvironment.transform.rotation * Quaternion.Inverse((Quaternion)spotTransform.aprilRotation);
            //dataTransform.transform.eulerAngles = dataTransform.transform.rotation.eulerAngles + new Vector3(-90.0f, 0f, 0f);
            //dataTransform.transform.eulerAngles = dataTransform.transform.rotation.eulerAngles + new Vector3(0f, 90.0f, 0f);
            //dataTransform.transform.eulerAngles = dataTransform.transform.rotation.eulerAngles + new Vector3(0f, 0f, 90.0f);

            // Move spot and marker to match the unity environment
            //Vector3 aprilTranslationPosition = aprilTagMarkerEnvironment.transform.position - aprilTagMarkerData.transform.position;
            //aprilTagMarkerData.transform.position = aprilTagMarkerData.transform.position + aprilTranslationPosition;
            //spotOffsetParentObject.transform.localPosition = aprilTranslationPosition;

        }
        else
        {
            aprilTagMarkerData.SetActive(false);
            aprilTagMarkerData.transform.localPosition = Vector3.zero;
            aprilTagMarkerData.transform.localRotation = Quaternion.identity;
            // Reset Spot's offset
            //spotOffsetParentObject.transform.localPosition = Vector3.zero;
            //spotOffsetParentObject.transform.localRotation = Quaternion.identity;
        }

        // Update body position
        spotModelBase.transform.localPosition = spotTransform.position;
        spotModelBase.transform.localRotation = spotTransform.rotation;

        // Update joints
        foreach (var joint in spotTransform.joints)
        {
            /*
            // Get model info
            string jointModelName = SpotJoint.jointNameMap[joint.name];
            GameObject modelJoint = GameObject.Find(jointModelName).gameObject;
            // Create euler angle
            float angle = joint.position; // Radians
            angle = Mathf.Rad2Deg * angle;
            Vector3 rot = new Vector3(0f, 0f, 0f);
            switch(SpotJoint.GetJointRotationAxis(jointModelName))
            {
                case Axis.X:
                    rot = Vector3.right * angle;
                    break;
                case Axis.Z:
                    rot = Vector3.forward * -angle;
                    break;
                default:
                    break;
            }
            */
            string jointModelName = SpotJoint.jointNameMap[joint.name];
            GameObject modelJoint = GameObject.Find(jointModelName).gameObject;
            Vector3 rot = SpotJoint.JointPositionToEulerAngles(joint);
            // Set model rotation
            modelJoint.transform.localEulerAngles = rot;
        }
        
        // Update line
        if(drawPathLine == true)
        {
            UpdatePathLine(desiredIndex);
        }
    }

    private void UpdatePathLine(int currentIndex)
    {
        spotLineRenderer.positionCount = 0; // Reset
        spotLineRenderer.positionCount = currentIndex;
        
        // TODO set all at once for better performance
        for (int i = 0; i < currentIndex; i++)
        {
            spotLineRenderer.SetPosition(i, spotTransforms[i].position);
        }
    }

    public void NextSpotFrame()
    {
        //int desiredIndex = ((int)playbackSlider.value + 1) % (int)playbackSlider.maxValue;
        int desiredIndex = Mathf.Min((int)playbackSlider.value + 1, (int)playbackSlider.maxValue);
        playbackSlider.value = desiredIndex;
        //UpdateSpotModel(); // Doesn't need to run, as update model already called by slider
    }

    public void PreviousSpotFrame()
    {
        int desiredIndex = Mathf.Max((int)playbackSlider.value - 1, (int)playbackSlider.minValue);
        playbackSlider.value = desiredIndex;
        //UpdateSpotModel();
    }

    /*
    private Axis GetJointRotationAxis(string jointUnityModelName)
    {
        if (jointUnityModelName.Contains("hip"))
        {
            return Axis.Z;
        }
        else if (jointUnityModelName.Contains("uleg"))
        {
            return Axis.X;
        }
        else if (jointUnityModelName.Contains("lleg"))
        {
            return Axis.X;
        }
        else
        {
            Debug.LogError("Joint " +  jointUnityModelName + " not able to map to a rotation axis");
            return Axis.None;
        }
    }
    */

    public void PlaybackButtonPressed()
    {
        if(playbackActive == true)
        {
            EndPlayback();
        }
        else
        {
            // Reset if slider is at end already
            if (playbackSlider.value >= playbackSlider.maxValue)
                playbackSlider.value = 0;
            StartPlayback();
        }
    }

    public IEnumerator NextSpotFrameLoop()
    {
        do
        {
            // Update model to next frame
            playbackSlider.value = Mathf.Min(playbackSlider.maxValue, playbackSlider.value + 1);
            UpdateSpotModel();
            //
            if(playbackSlider.value >= playbackSlider.maxValue)
                EndPlayback();
            // Wait
            yield return new WaitForSeconds(1.0f / ((float)UPDATE_FPS));
        } while (playbackActive == true);
    }

    private void StartPlayback()
    {
        playbackActive = true;
        playbackButton.GetComponentInChildren<TextMeshProUGUI>().text = "ll";
        // Start playback
        playbackCoroutine = NextSpotFrameLoop();
        StartCoroutine(playbackCoroutine);
    }

    private void EndPlayback()
    {
        playbackActive = false;
        playbackButton.GetComponentInChildren<TextMeshProUGUI>().text = ">";
        playbackCoroutine = null; // Should die on it's own because playbackActive is now false
    }
}
