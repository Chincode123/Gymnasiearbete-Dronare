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


    [SerializeField] float a_p, a_i, a_d;
    float angleIntegral = 0, previousAngleError = 0;
    [SerializeField] float desiredAngle;

    [SerializeField] float v_p, v_i, v_d;
    float velocityIntegral = 0, previousVelocityError = 0;
    [SerializeField] float desiredVelocity;

    // 0-vector is up
    Vector3 direction;

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
        desiredVelocity = 10 * Input.GetAxisRaw("Triggers") * Mathf.Abs(Input.GetAxisRaw("Triggers"));

        desiredAngle = (-Mathf.PI / 8) * Input.GetAxisRaw("Vertical");

        ShiftPropellerPower();

    }

    void FixedUpdate()
    {

        for (int i = 0; i < 4; i++)
        {
            rigidbody.AddForceAtPosition(transform.up * propellerPower[i], propellers[i].position);
        }

        UpdateGyroscopeValues(Time.fixedDeltaTime);
    }


    // [Get from mpu]
    void UpdateGyroscopeValues(float timeIncrement)
    {
        acceleration = (rigidbody.velocity - previousVelocity) / timeIncrement;
        previousVelocity = rigidbody.velocity;

        gyro = rigidbody.angularVelocity * Mathf.Rad2Deg;
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
        float velocityErorr = desiredVelocity - Vector3.Project(rigidbody.velocity, Vector3.up).y;

        velocityIntegral += velocityErorr * Time.deltaTime;

        float velocityDerivitive = (velocityErorr - previousVelocityError) / Time.deltaTime;
        previousVelocityError = velocityErorr;

        float basePower = v_p * velocityErorr + v_i * velocityIntegral + v_d * velocityDerivitive;

        basePower = Mathf.Clamp(basePower, 0, maxPropellerPower);

        desiredAngle = Mathf.Clamp(desiredAngle, -Mathf.PI/2, Mathf.PI/2);
        float angleError = desiredAngle - Mathf.Atan2(-transform.up.z, transform.up.y);

        angleIntegral += angleError * Time.deltaTime;

        float angleDerivitive = (angleError - previousAngleError) / Time.deltaTime;
        previousAngleError = angleError;

        float shift = a_p * angleError + a_i * angleIntegral + a_d * angleDerivitive;

        propellerPower[0] = basePower + shift;
        propellerPower[1] = basePower + shift;
        propellerPower[2] = basePower - shift;
        propellerPower[3] = basePower - shift;

        for(int i = 0; i < 4; i++)
        {
            propellerPower[i] = Mathf.Clamp(propellerPower[i], 0, maxPropellerPower);
        }
    }

    float map(float a, float min, float max, float nMin, float nMax)
    {
        return (a / (max - min)) * (nMax - nMin);
    }

    void OnDrawGizmos()
    {
        // propeller forces
        for (int i = 0; i < 4; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(propellers[i].position, propellers[i].position + propellerPower[i] * propellers[i].up);
        }

        // velocitiy
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + rigidbody.velocity);

        // acceleration
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(transform.position, transform.position + acceleration);
    }
}