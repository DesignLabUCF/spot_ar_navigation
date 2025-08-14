using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

public class ParticipantLogger : MonoBehaviour
{
    private string participantID = "UNSET";
    private List<DataPoint> dataPoints;

    private string baseOutputDirectory = "OutputData";
    private string timestampFormat = "MM-dd-yyyy_HH-mm-ss-fff";

    private void Awake()
    {
        dataPoints = new List<DataPoint>();

        // Ensure base directory exists
        /*
        if (!Directory.Exists(Path.Join(Application.persistentDataPath, baseOutputDirectory)))
            Directory.CreateDirectory(Path.Join(Application.persistentDataPath, baseOutputDirectory));
        */
    }

    private void Start()
    {

    }

    private void Update()
    {
        /*
        if (Input.GetKeyDown(KeyCode.H))
            AddGoToPoint(new Vector2(1.0f, 0f), false);
        if (Input.GetKeyDown(KeyCode.L))
            AddStop(true);
        */
    }

    public void SetParticipantID(string participantID)
    {
        Regex rgx = new Regex("[^a-zA-Z0-9 -]");
        participantID = rgx.Replace(participantID, "");
        this.participantID = participantID;
    }

    public string GetParticipantID()
    {
        return participantID;
    }

    public void AddGoToPoint(Vector3 pointOffset, bool isInverted)
    {
        Debug.Log("Adding GoToDataPoint to data log");
        dataPoints.Add(new GoToDataPoint(participantID, pointOffset, isInverted));
        TriggerAutosave();
    }

    public void AddStop(bool stopped)
    {
        Debug.Log("Adding GoToDataPoint to data log");
        dataPoints.Add(new StopDataPoint(participantID, stopped));
        TriggerAutosave();
    }

    public void AddParticipantHandoff()
    {
        Debug.Log("Adding HeadsetExchanged to data log");
        dataPoints.Add(new ParticipantHandoffDataPoint(participantID));
        TriggerAutosave();
    }

    public void AddTutorialComplete()
    {
        Debug.Log("Adding TutorialComplete to data log");
        dataPoints.Add(new TutorialCompleteDataPoint(participantID));
        TriggerAutosave();
    }

    private void TriggerAutosave()
    {
        StartCoroutine(Save());
    }

    private IEnumerator Save()
    {
        // Setup file name params
        string fileDirectory = Path.Join(Application.persistentDataPath, baseOutputDirectory, participantID);
        string filePath = Path.Join(fileDirectory, DateTime.Now.ToString(timestampFormat)) + ".csv";
        //string filePath = Path.Join("C:\\", DateTime.Now.ToString(timestampFormat)) + ".csv";
        // Create Directory if needed
        if (!Directory.Exists(fileDirectory))
            Directory.CreateDirectory(fileDirectory);
        Debug.Log("Saving to file: " + filePath);
        // Create and write to file
        StreamWriter writer = new StreamWriter(filePath);
        List<DataPoint> lockedList = dataPoints;
        writer.WriteLine(DataPoint.GetDataHeader()); // Header
        foreach (DataPoint point in lockedList)
        {
            writer.WriteLine(point.GetRowData());
        }
        writer.Close();
        yield return null;
    }
}

public class DataPoint
{
    protected DateTime timestamp = DateTime.MinValue;
    protected string participantID = "";
    protected string dataType = "";
    protected string timestampFormat = "MM-dd-yyyy_HH-mm-ss-fff";


    public DataPoint(string participantID, string dataType)
    {
        timestamp = DateTime.Now;
        this.participantID = participantID;
        this.dataType = dataType;
    }

    public virtual string GetRowData()
    {
        return participantID + "," + timestamp.ToString(timestampFormat) + "," + this.dataType;
    }

    public static string GetDataHeader()
    {
        return "Participant.ID,Timestamp,Data.Type,Data";
    }
}

public class GoToDataPoint : DataPoint
{
    protected Vector3 pointOffset = Vector3.zero;
    protected bool isInverted = false;

    public GoToDataPoint(string participantID, Vector3 pointOffset, bool isInverted) : base(participantID, "GoToOffset")
    {
        this.pointOffset = pointOffset;
        this.isInverted = isInverted;
    }

    public override string GetRowData()
    {
        //return base.GetRowData() + "," + this.isInverted.ToString() + "," + this.pointOffset.ToString("F6");
        return base.GetRowData() + "," + this.isInverted.ToString() + "," + pointOffset.x + "," + pointOffset.y + "," + pointOffset.z;
    }
}

public class StopDataPoint: DataPoint
{
    protected bool stopped;

    public StopDataPoint(string participantID, bool stopped) : base(participantID, "Stop")
    {
        this.stopped = stopped;
    }
    public override string GetRowData()
    {
        return base.GetRowData() + "," + this.stopped.ToString();
    }
}

public class ParticipantHandoffDataPoint : DataPoint
{
    public ParticipantHandoffDataPoint(string participantID) : base(participantID, "HeadsetExchanged")
    {
        ;
    }
    public override string GetRowData()
    {
        return base.GetRowData();
    }
}

public class TutorialCompleteDataPoint : DataPoint
{
    public TutorialCompleteDataPoint(string participantID) : base(participantID, "TutorialComplete")
    {
        ;
    }
    public override string GetRowData()
    {
        return base.GetRowData();
    }
}
