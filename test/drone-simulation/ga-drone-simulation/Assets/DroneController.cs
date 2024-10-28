using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    Vector3 d_accel, d_gyro;
    Vector3 previousVelocity;

    [SerializeField] float maxPropellerPower;
    [SerializeField] float[] propellerPower = new float[4];
    [SerializeField] Transform[] propellers = new Transform[4];
    [SerializeField] Vector3[] propellerDirections = new Vector3[4];

    // 0-vector is up
    Vector3 direction;

    [SerializeField] new Rigidbody rigidbody;

    private void Start()
    {
        for (int i = 0; i< 4; i++)
        {
            propellerDirections[i] = propellers[i].position - transform.position;
        }
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

    void UpdateGyroscopeValues(float timeIncrement)
    {
        d_accel = (rigidbody.velocity - previousVelocity) / timeIncrement;
        previousVelocity = rigidbody.velocity;

        d_gyro = rigidbody.angularVelocity * Mathf.Rad2Deg;
    }

    void ShiftPropellerPower()
    {
        for (int i = 0; i < 4; i++)
        {
            //int sign = (Vector3.Dot(transform.up, propellerDirections[i]) < 0) ? -1 : 1;
            //float fsign = sign * Vector3.Project(transform.up, propellerDirections[i]).magnitude;


            //int up_down = (Vector3.Project(rigidbody.velocity, transform.up).y < 0) ? 1 : -1;

            //float shift = (up_down * rigidbody.velocity.magnitude + Mathf.LerpUnclamped(0, 1, d_gyro.sqrMagnitude / 1000000) * sign / Time.deltaTime) *Time.deltaTime;   
            //propellerPower[i] += shift;


            //Debug.Log(i + ": gyro: " + d_gyro + " m:" + Mathf.LerpUnclamped(-1, 1, d_gyro.sqrMagnitude / 1000000) + " acc: " + d_accel + " m:" + d_accel.magnitude + " fsign: " + fsign + " shift: " + shift);


            //int velocity_dir = (Vector3.Dot(transform.up, rigidbody.velocity) < 0) ? 1 : -1;
            //int acceleration_dir = (Vector3.Dot(transform.up, d_accel) < 0) ? 1 : -1;
            //int rotation_dir = (Vector3.Dot(transform.up, propellerDirections[i]) < 0) ? 1 : -1;

            //float velocity_importance = Vector3.Project(transform.up, rigidbody.velocity).magnitude;
            //float acceleration_importance = Vector3.Project(transform.up, d_accel).magnitude;
            //float rotaiton_importance = Vector3.Project(transform.up, propellerDirections[i]).magnitude;

            //float calculated_velocity = velocity_dir * velocity_importance;
            //float calculated_acceleration = acceleration_importance + acceleration_dir * 2;
            //float calculated_rotation = rotation_dir * rotaiton_importance * 5;

            //float shift = maxPropellerPower * ((calculated_velocity + acceleration_dir * calculated_acceleration * calculated_rotation) / 8) * Time.deltaTime;
            //propellerPower[i] += shift;

            //Debug.Log(i + ": vDir:" + velocity_dir + " vImp:" + velocity_importance + " vCalc:" + calculated_velocity +
            //    " aDir:" + acceleration_dir + " aImp:" + acceleration_importance + " aCalc:" + calculated_acceleration +
            //    " rDir:" + rotation_dir + " rImp:" + rotaiton_importance + " rCalc:" + calculated_rotation + " shift:" + shift);


            
            
            Vector3 projected_acceleration = Vector3.Project(d_accel, transform.up);
            if (Mathf.Abs(projected_acceleration.y) < 0.01f)
            {
                Vector3 projected_velocity = Vector3.Project(rigidbody.velocity, transform.up);
                propellerPower[i] -= projected_velocity.y * Time.deltaTime;
            }
            else
                propellerPower[i] -= projected_acceleration.y * Time.deltaTime;






            float shift = Vector3.Project(transform.up, propellerDirections[i]).sqrMagnitude * ((Vector3.Dot(transform.up, propellerDirections[i]) < 0) ? -.1f : 1) * d_gyro.sqrMagnitude * Time.deltaTime;
            propellerPower[i] += shift;












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
        Gizmos.DrawLine(transform.position, transform.position + d_accel);

        // dir
        Gizmos.color = Color.cyan;

        Vector3 u = Vector3.ProjectOnPlane(transform.up, Vector3.up);
        float angle = Mathf.Atan2(u.z, u.x) + transform.rotation.eulerAngles.y * Mathf.Deg2Rad;
        Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));

        Gizmos.DrawLine(transform.position, transform.position + direction);
    }
}
