using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    Vector3 acceleration, gyro, d_gyro;
    Vector3 previousVelocity, previousGyro;

    [SerializeField] float maxPropellerPower;
    [SerializeField] float[] propellerPower = new float[4];
    [SerializeField] Transform[] propellers = new Transform[4];
    [SerializeField] Vector3[] propellerDirections = new Vector3[4];

    Vector3[] previousPorpellerPositions = new Vector3[4];
    Vector3[] propellerVelocities = new Vector3[4];
    Vector3[] previousPropellerVelocities = new Vector3[4];
    Vector3[] propellerAccelerations = new Vector3[4];


    [SerializeField] float pitchP, pitchI, pitchD;
    float pitchIntegral = 0, previousPitchError = 0;
    [SerializeField] float desiredPitch;

    [SerializeField] float rollP, rollI, rollD;
    float rollIntegral = 0, previousRollError = 0;
    [SerializeField] float desiredRoll;

    [SerializeField] float velocityP, velocityI, velocityD;
    float velocityIntegral = 0, previousVelocityError = 0;
    [SerializeField] float desiredVelocity;

    [SerializeField] float angleP, angleI, angleD;

    [SerializeField] float velocityRange, pitchRange, rollRange;
    [SerializeField] float currentPitch, currentRoll;

    Vector3 rotation;

    [SerializeField] new Rigidbody rigidbody;

    private void Start()
    {
        for (int i = 0; i< 4; i++)
        {
            propellerDirections[i] = propellers[i].position - transform.position;
        }


        //Time.timeScale = 0.1f;
    }

    void Update()
    {
        desiredVelocity = velocityRange * Input.GetAxisRaw("Triggers") * Mathf.Abs(Input.GetAxisRaw("Triggers"));
        desiredPitch = -pitchRange * Input.GetAxisRaw("Vertical");
        desiredRoll = -rollRange * Input.GetAxisRaw("Horizontal");

        // currentPitch = Mathf.Atan2(-transform.up.z, transform.up.y) * Mathf.Rad2Deg;
        // currentRoll = Mathf.Atan2(-transform.up.x, transform.up.y)  * Mathf.Rad2Deg;

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
            rigidbody.AddForceAtPosition(transform.up * calculateMotorPower((propellerPower[i] / 255) + 0.5f), propellers[i].position);
        }

        UpdateGyroscopeValues(Time.fixedDeltaTime);
    }


    // [Get from mpu]
    void UpdateGyroscopeValues(float timeIncrement)
    {
        acceleration = (rigidbody.velocity - previousVelocity) / timeIncrement;
        previousVelocity = rigidbody.velocity;

        gyro = rigidbody.angularVelocity * Mathf.Rad2Deg;
        rotation += gyro * timeIncrement;
        d_gyro = (gyro - previousGyro) / timeIncrement;
        previousGyro = gyro;


        // propelers
        for (int i = 0; i < 4; i++)
        {
            propellerVelocities[i] = (propellers[i].position - previousPorpellerPositions[i]) / timeIncrement;
            previousPorpellerPositions[i] = propellers[i].position;

            propellerAccelerations[i] = (propellerVelocities[i] - previousPropellerVelocities[i]) / timeIncrement;
            previousPropellerVelocities[i] = propellerVelocities[i];

            //// velocities
            //Debug.DrawLine(propellers[i].position, propellers[i].position + propellerVelocities[i], Color.blue);
            //// accelerations
            //Debug.DrawLine(propellers[i].position, propellers[i].position + propellerAccelerations[i], Color.red);
        }
    }

    void ShiftPropellerPower()
    {
        float velocityErorr = desiredVelocity - rigidbody.velocity.y;

        velocityIntegral += velocityErorr * Time.deltaTime;

        float velocityDerivitive = (velocityErorr - previousVelocityError) / Time.deltaTime;
        previousVelocityError = velocityErorr;

        float basePower = velocityP * velocityErorr + velocityI * velocityIntegral + velocityD * velocityDerivitive;
        basePower = Mathf.Clamp(basePower, -127, 127);

        float pitchError = desiredPitch - currentPitch;

        pitchIntegral += pitchError * Time.deltaTime;

        float pitchDerivitive = (pitchError - previousPitchError) / Time.deltaTime;
        previousPitchError = pitchError;



        float pitchShift = pitchP * pitchError + pitchI * pitchIntegral + pitchD * pitchDerivitive;



        float rollError = desiredRoll - currentRoll;

        pitchIntegral += rollError * Time.deltaTime;

        float rollDerivitive = (rollError - previousRollError) / Time.deltaTime;
        previousRollError = rollError;

        float rollShift = rollP * rollError + rollI * rollIntegral + rollD * rollDerivitive;



        propellerPower[0] = basePower + pitchShift + rollShift;
        propellerPower[1] = basePower + pitchShift - rollShift;
        propellerPower[2] = basePower - pitchShift - rollShift;
        propellerPower[3] = basePower - pitchShift + rollShift;


        for (int i = 0; i < 4; i++)
        {
            propellerPower[i] = Mathf.Clamp(propellerPower[i], -127, 127);
        }
    }

    float map(float x, float min, float max, float nMin, float nMax)
    {
        return (x / (max - min)) * (nMax - nMin);
    }

    // 0 <= x <= 1
    float calculateMotorPower(float x) {
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
            Gizmos.DrawLine(propellers[i].position, propellers[i].position + (propellerPower[i] / 25) * propellers[i].up);
        }

        // velocitiy
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + rigidbody.velocity);

        // acceleration
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(transform.position, transform.position + acceleration);
    }
}