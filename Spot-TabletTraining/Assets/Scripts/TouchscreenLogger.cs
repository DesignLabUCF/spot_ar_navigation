using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class TouchscreenLogger : MonoBehaviour, IPointerDownHandler, IPointerClickHandler,
    IPointerUpHandler, IPointerExitHandler, IPointerEnterHandler,
    IBeginDragHandler, IDragHandler, IEndDragHandler
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        Debug.Log("Mouse Down: " + eventData.position);
    }

    public void OnBeginDrag(PointerEventData eventData)
    {
        ;
    }

    public void OnDrag(PointerEventData eventData)
    {
        ;
    }

    public void OnEndDrag(PointerEventData eventData)
    {
        ;
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        ;
    }


    public void OnPointerEnter(PointerEventData eventData)
    {
        ;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        ;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        ;
    }
}
