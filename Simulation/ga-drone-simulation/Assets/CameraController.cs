using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] Transform drone;

    Vector3 smoothPosition;

    void Update() {
        smoothPosition = smoothPosition * 0.9f + drone.position * 0.1f;;
    }

    void LateUpdate()
    {
        transform.LookAt(smoothPosition);
    }
}
