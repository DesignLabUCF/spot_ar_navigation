using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TutorialTarget : MonoBehaviour
{
    private TutorialManager tutorialManager;

    private void Start()
    {
        tutorialManager = transform.parent.GetComponent<TutorialManager>();
    }

    void Update()
    {
        
    }

    public void TargetSelected()
    {
        tutorialManager.MarkTarget(this.gameObject);
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log(other.tag);
        if(other.tag.ToUpper().Contains("PLAYER"))
        {
            TargetSelected();
        }
    }
}
