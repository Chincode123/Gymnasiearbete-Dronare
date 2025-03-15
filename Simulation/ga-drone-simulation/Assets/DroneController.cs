using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    [Serializable] struct PIDConstants
    {
        public float P;
        public float I;
        public float D;

        public PIDConstants(float p, float i, float d)
        {
            P = p;
            I = i;
            D = d;
        }
    }

    [Header("Propeller Info")]
    [SerializeField] float[] propellerPower = new float[4];
    [SerializeField] Transform[] propellers = new Transform[4];
    Vector3[] previousPorpellerPositions = new Vector3[4];
    Vector3[] propellerVelocities = new Vector3[4];
    Vector3[] previousPropellerVelocities = new Vector3[4];
    Vector3[] propellerAccelerations = new Vector3[4];

    [Header("Pitch")]
    [SerializeField] float desiredPitch;
    [SerializeField] PIDConstants pitchPID;
    float pitchIntegral = 0, previousPitchError = 0;

    [Header("Roll")]
    [SerializeField] float desiredRoll;
    [SerializeField] PIDConstants rollPID;
    float rollIntegral = 0, previousRollError = 0;

    [Header("Velocity")]
    [SerializeField] float desiredVelocity;
    [SerializeField] PIDConstants velocityPID;
    float velocityIntegral = 0, previousVelocityError = 0;


    [Header("Ranges")]
    [SerializeField] float velocityRange;
    [SerializeField] float pitchRange;
    [SerializeField] float rollRange;

    [Header("Info")]
    [SerializeField] float currentPitch;
    [SerializeField] float currentRoll;
    [SerializeField] float currentVerticalVelocity;

    [Header("Center of Mass")]
    [SerializeField] Vector3 centerOfMass;

    Vector3 rotation;

    [Header("References")]
    [SerializeField] new Rigidbody rigidbody;

    private void Start()
    {
        rigidbody.centerOfMass = centerOfMass;
    }

    void Update()
    {
        float inputPower = ((Input.GetKey(KeyCode.LeftShift)) ? 0 : 1) - ((Input.GetKey(KeyCode.Space)) ? 0 : 1);
        inputPower = (inputPower != 0) ? inputPower : Input.GetAxisRaw("Triggers") * Mathf.Abs(Input.GetAxisRaw("Triggers")) ;
        desiredVelocity = velocityRange * inputPower;
        desiredPitch = -pitchRange * Input.GetAxisRaw("Vertical");
        desiredRoll = -rollRange * Input.GetAxisRaw("Horizontal");

        currentVerticalVelocity = rigidbody.velocity.y;
        currentPitch = rigidbody.rotation.eulerAngles.x;
        currentRoll = rigidbody.rotation.eulerAngles.z;
        // Adjust angles to -180 to 180 range
        currentPitch = currentPitch > 180 ? -(currentPitch - 360) : -currentPitch;
        currentRoll = currentRoll > 180 ? currentRoll - 360 : currentRoll;

        ShiftPropellerPower();
    }

    void FixedUpdate()
    {
        for (int i = 0; i < 4; i++)
        {
                                                                            // -0.5 <= (power / 255) <= 0.5
            rigidbody.AddForceAtPosition(propellers[i].up * CalculateMotorPower((propellerPower[i] / 255) + 0.5f), propellers[i].position);
        }
    }

    void ShiftPropellerPower()
    {
        float velocityErorr = desiredVelocity - currentVerticalVelocity;
        velocityIntegral += velocityErorr * Time.deltaTime;
        float velocityDerivitive = (velocityErorr - previousVelocityError) / Time.deltaTime;
        previousVelocityError = velocityErorr;
        float basePower = velocityPID.P * velocityErorr + velocityPID.I * velocityIntegral + velocityPID.D * velocityDerivitive;
        basePower = Mathf.Clamp(basePower, -127, 127);

        float pitchError = desiredPitch - currentPitch;
        pitchIntegral += pitchError * Time.deltaTime;
        float pitchDerivitive = (pitchError - previousPitchError) / Time.deltaTime;
        previousPitchError = pitchError;
        float pitchShift = pitchPID.P * pitchError + pitchPID.I * pitchIntegral + pitchPID.D * pitchDerivitive;

        float rollError = desiredRoll - currentRoll;
        rollIntegral += rollError * Time.deltaTime;
        float rollDerivitive = (rollError - previousRollError) / Time.deltaTime;
        previousRollError = rollError;
        float rollShift = rollPID.P * rollError + rollPID.I * rollIntegral + rollPID.D * rollDerivitive;

        propellerPower[0] = basePower + pitchShift + rollShift;
        propellerPower[1] = basePower + pitchShift - rollShift;
        propellerPower[2] = basePower - pitchShift - rollShift;
        propellerPower[3] = basePower - pitchShift + rollShift;

        for (int i = 0; i < 4; i++)
            propellerPower[i] = Mathf.Clamp(propellerPower[i], -127, 127);
    }

    // 0 <= x <= 1
    float CalculateMotorPower(float x) {
        if (x <= 0)
            return 0;
        
        if (x > 1)
            x = 1;
        
        // function generated from regression of provided motor powers (https://darwinfpv.com/products/darwin-1104-4300kv-brushless-motor) using Geogebra (https://www.geogebra.org/m/vqm62ynm)
        return (float)(-2.7731729055258 * (x * x * x * x) + 4.2924375448958 * (x * x * x) - 0.9893438555883 * (x * x) + 0.9857510176391 * x - 0.0011584164738);
    }

    void OnDrawGizmos()
    {
        // propeller forces
        for (int i = 0; i < 4; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(propellers[i].position, propellers[i].position + (propellerPower[i] / 100) * propellers[i].up);
        }

        // velocitiy
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + rigidbody.velocity);

        // Center of mass
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(transform.position + transform.rotation * centerOfMass, 0.01f);
    }
}