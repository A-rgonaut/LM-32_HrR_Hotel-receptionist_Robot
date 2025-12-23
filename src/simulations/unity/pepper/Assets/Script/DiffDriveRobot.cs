using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using System.Collections.Generic;
using System.Globalization;

public class DiffDriveRobot : MonoBehaviour
{
    [Header("Componenti Robot")]
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    [Header("Parametri ROS")]
    public string leftWheelCmdTopic = "/left_wheel_cmd";
    public string rightWheelCmdTopic = "/right_wheel_cmd";
    public string wheelsStateTopic = "/wheels_state";
    public float publishRate = 10f;

    private ROSConnection ros;
    private float timeElapsed;

    void Awake()
    {
        // Fix per i decimali (punto vs virgola)
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.DefaultThreadCurrentUICulture = CultureInfo.InvariantCulture;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Sottoscrizioni
        ros.Subscribe<Float64Msg>(leftWheelCmdTopic, LeftWheelCmdCallback);
        ros.Subscribe<Float64Msg>(rightWheelCmdTopic, RightWheelCmdCallback);

        // Publisher
        ros.RegisterPublisher<JointStateMsg>(wheelsStateTopic);

        Debug.Log("DiffDriveRobot avviato. In attesa di comandi ROS...");
    }

    void FixedUpdate()
    {
        timeElapsed += Time.fixedDeltaTime;
        if (timeElapsed >= 1.0f / publishRate)
        {
            PublishWheelStates();
            timeElapsed = 0;
        }
    }

    void LeftWheelCmdCallback(Float64Msg msg)
    {
        // LOG: Vediamo se arriva il comando
        Debug.Log($"[ROS -> Unity] CMD Sinistra: {msg.data} rad/s");
        SetSpeed(leftWheel, (float)msg.data);
    }

    void RightWheelCmdCallback(Float64Msg msg)
    {
        // LOG: Vediamo se arriva il comando
        Debug.Log($"[ROS -> Unity] CMD Destra: {msg.data} rad/s");
        SetSpeed(rightWheel, (float)msg.data);
    }

    void SetSpeed(ArticulationBody wheel, float speedRadS)
    {
        // NOTA BENE: Qui assumiamo che la ruota giri sull'asse X (xDrive).
        // Se la tua ruota è impostata su Z nell'editor, questo codice non funzionerà.
        // Vedi punto "2. Configurazione Fisica" sotto.

        var drive = wheel.xDrive;
        drive.targetVelocity = speedRadS * Mathf.Rad2Deg; // Conversione rad/s -> deg/s
        wheel.xDrive = drive;
    }

    void PublishWheelStates()
    {
        JointStateMsg stateMsg = new JointStateMsg();
        stateMsg.name = new string[] { "right_wheel", "left_wheel" };

        // Prende la velocità attuale (in rad/s, Unity la fornisce già convertita se configurato bene)
        double velRight = rightWheel.jointVelocity[0];
        double velLeft = leftWheel.jointVelocity[0];

        stateMsg.velocity = new double[] { velRight, velLeft };

        ros.Publish(wheelsStateTopic, stateMsg);

        // LOG: Decommenta se vuoi vedere cosa esce (può intasare la console)
        // Debug.Log($"[Unity -> ROS] Encoder: L={velLeft:F2}, R={velRight:F2}");
    }
}