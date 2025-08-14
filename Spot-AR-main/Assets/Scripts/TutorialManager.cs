using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TutorialManager : MonoBehaviour
{
    public ParticipantLogger participantLogger;

    private List<GameObject> targets;
    private int targetsSelected = 0;
    private int targetCount = 0;

    private void Awake()
    {
        targets = new List<GameObject>();
    }

    private void Start()
    {
        // Get targets for tracking progress
        for(int i = 0; i < transform.childCount; i++)
        {
            if(transform.GetChild(i).GetComponent<TutorialTarget>() != null)
            {
                targets.Add(transform.GetChild(i).gameObject);
            }
        }
        targetCount = targets.Count;
    }

    public void MarkTarget(GameObject target)
    {
        // Remove target from scene
        targets.Remove(target);
        target.SetActive(false);
        // Log
        targetsSelected = targetsSelected + 1;
        Debug.Log("Tutorial targets selected: " + targetsSelected + "/" + targetCount);
        // Check if finished
        if(targets.Count <= 0)
        {
            EndTutorial();
        }
    }

    public void EndTutorial()
    {
        Debug.Log("Tutorial complete");
        participantLogger.AddTutorialComplete();
        gameObject.SetActive(false);
    }
}
