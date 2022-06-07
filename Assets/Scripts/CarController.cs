using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class CarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos; // the information about each individual axle
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle;// maximum steer angle the wheel can have
    public float MotorTorqueMulti;
    public float motorspeed;
    public float currentbreakForce;
    public float breakForce;
    public bool IsBraking;
    public float speed;

    public  TMP_Text Speedometer;
    public GameObject COM;
    public Rigidbody rb;
   

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = COM.transform.localPosition;
       // Speedometer = GameObject.Find("Speedometer").GetComponent<Text>();
    }



    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        rotation *= Quaternion.Euler(0, 0, 90);


        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }
    public void Brake(AxleInfo axleInfo)
    {

        if (IsBraking == true)
        {



            axleInfo.leftWheel.brakeTorque = currentbreakForce;
            axleInfo.rightWheel.brakeTorque = currentbreakForce;





        }
      


    }

    public void FixedUpdate()
    {
        motorspeed = maxMotorTorque * MotorTorqueMulti * 100;
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
       
       

        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motorspeed;
                axleInfo.rightWheel.motorTorque = motorspeed;
            }

            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);


            if (Input.GetKey(KeyCode.Space))
            {
                currentbreakForce = breakForce;
                Brake(axleInfo);
                IsBraking = true;
            }
            else 
            {
                axleInfo.leftWheel.brakeTorque = 0;
                axleInfo.rightWheel.brakeTorque = 0;

            }
            Debug.Log("braking: " + IsBraking + axleInfo.rightWheel.brakeTorque + axleInfo.rightWheel.brakeTorque);
        }

        rb.velocity = Vector3.ClampMagnitude(rb.velocity, 41.0f);
        var speedm = rb.velocity.magnitude;
        speed = speedm * 3.6f * 0.621371f;
    
        Speedometer.text = (speed.ToString("f1") + "mp/h");

    }




}

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor; // is this wheel attached to motor?
    public bool steering; // does this wheel apply steer angle?
}
