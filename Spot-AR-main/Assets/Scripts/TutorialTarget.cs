using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TutorialTarget : MonoBehaviour
{
    private TutorialManager tutorialManager;

    public Material downMaterial;

    private void Start()
    {
        tutorialManager = transform.parent.GetComponent<TutorialManager>();
    }

    public void TargetSelected()
    {
        tutorialManager.MarkTarget(this.gameObject);
    }

    public void TargetDown()
    {
        GetComponent<MeshRenderer>().material = downMaterial;
    }

    /*
    public EventHandler<GameObject> targetSelected;

    public void TargetSelected()
    {
        targetSelected.Invoke(this, this.gameObject);
    }
    */

}
