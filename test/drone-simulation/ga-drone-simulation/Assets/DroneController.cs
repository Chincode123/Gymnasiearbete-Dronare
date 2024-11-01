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

        if (Input.GetAxisRaw("Triggers") != 0)
        {
            for (int i = 0; i < 4; i++)
            {
                propellerPower[i] += maxPropellerPower * Input.GetAxisRaw("Triggers") * Time.deltaTime;
            }
        }

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

            // velocities
            Debug.DrawLine(propellers[i].position, propellers[i].position + propellerVelocities[i], Color.blue);
            // accelerations
            Debug.DrawLine(propellers[i].position, propellers[i].position + propellerAccelerations[i], Color.red);
        }
    }

    void ShiftPropellerPower()
    {
        for (int i = 0; i < 4; i++)
        {                    
            //Vector3 projected_acceleration = Vector3.Project(acceleration, transform.up);
            //if (Mathf.Abs(projected_acceleration.y) < 0.1f)
            //{
            //    Vector3 projected_velocity = Vector3.Project(rigidbody.velocity, transform.up);
            //    propellerPower[i] -= projected_velocity.y * Time.deltaTime * 10;
            //}
            //else
            //    propellerPower[i] -= projected_acceleration.y * Time.deltaTime;



            float shift = 0;


            Vector3 projected_acceleration = Vector3.Project(propellerAccelerations[i], transform.up);
            int acceleration_direction = (Vector3.Dot(projected_acceleration, transform.up) < 0) ? -1 : 1;
            if (projected_acceleration.magnitude < 0.3f)
            {
                Vector3 projected_velocity = Vector3.Project(propellerVelocities[i], transform.up);
                int velocity_direction = (Vector3.Dot(projected_velocity, transform.up) < 0) ? -1 : 1;
                propellerPower[i] -= projected_velocity.magnitude * velocity_direction * Time.deltaTime * 10;
            }
            else
                propellerPower[i] -= projected_acceleration.magnitude * acceleration_direction * Time.deltaTime;




            shift *= Time.deltaTime;
            propellerPower[i] += shift;


            Debug.Log(shift);










            propellerPower[i] = Mathf.Clamp(propellerPower[i], 0, maxPropellerPower);
        }
    }

    float map(float a, float min, float max, float nMin, float nMax)
    {
        return (a / (max - min)) * (nMax - nMin);
    }

    //void OnDrawGizmos()
    //{
    //    // propeller forces
    //    for (int i = 0; i < 4; i++)
    //    {
    //        Gizmos.color = Color.red;
    //        Gizmos.DrawLine(propellers[i].position, propellers[i].position + propellerPower[i] * propellers[i].up);
    //    }

    //    // velocitiy
    //    Gizmos.color = Color.yellow;
    //    Gizmos.DrawLine(transform.position, transform.position + rigidbody.velocity);

    //    // acceleration
    //    Gizmos.color = Color.magenta;
    //    Gizmos.DrawLine(transform.position, transform.position + acceleration);
    //}
}
