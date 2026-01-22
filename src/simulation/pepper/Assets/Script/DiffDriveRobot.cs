using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using System.Collections.Generic;
using System.Globalization;
using System;

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

    [Header("Safety Watchdog")]
    public float commandTimeout = 0.5f;

    [Tooltip("Decelerazione in rad/s^2. Valori bassi = frenata dolce.")]
    public float brakingDeceleration = 2.0f;

    // --- NUOVI PARAMETRI FISICI ---
    [Header("Fisica Ruote (Tuning)")]
    [Tooltip("La coppia massima (forza) che il motore può applicare. Aumenta se il robot fatica a partire o fermarsi.")]
    public float wheelForceLimit = 1000f; // Default alto per massa 28kg

    [Tooltip("Quanto velocemente la ruota cerca di raggiungere la velocità target. Più è alto, più è reattivo.")]
    public float wheelDamping = 500f;

    [Tooltip("Per controllo velocità deve essere 0.")]
    public float wheelStiffness = 0f;
    // -----------------------------

    private float lastCmdReceivedTime;
    private bool isStopped = false;

    private ROSConnection ros;
    private float timeElapsed;

    void Awake()
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.DefaultThreadCurrentUICulture = CultureInfo.InvariantCulture;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(leftWheelCmdTopic, LeftWheelCmdCallback);
        ros.Subscribe<Float64Msg>(rightWheelCmdTopic, RightWheelCmdCallback);
        ros.RegisterPublisher<JointStateMsg>(wheelsStateTopic);

        // Inizializza subito i parametri fisici delle ruote
        ApplyDriveParameters(leftWheel);
        ApplyDriveParameters(rightWheel);

        lastCmdReceivedTime = Time.time;
        Debug.Log("DiffDriveRobot avviato.");
    }

    void OnDestroy()
    {
        ros.Disconnect();
    }

    void FixedUpdate()
    {
        CheckConnectionWatchdog();

        timeElapsed += Time.fixedDeltaTime;
        if (timeElapsed >= 1.0f / publishRate)
        {
            PublishWheelStates();
            timeElapsed = 0;
        }
    }

    void CheckConnectionWatchdog()
    {
        if (Time.time - lastCmdReceivedTime > commandTimeout)
        {
            if (!isStopped)
            {
                // LOGICA DI FRENATA PROGRESSIVA
                float currentLeftVel = leftWheel.xDrive.targetVelocity * Mathf.Deg2Rad;
                float currentRightVel = rightWheel.xDrive.targetVelocity * Mathf.Deg2Rad;

                float step = brakingDeceleration * Time.fixedDeltaTime;

                float newLeftVel = Mathf.MoveTowards(currentLeftVel, 0f, step);
                float newRightVel = Mathf.MoveTowards(currentRightVel, 0f, step);

                SetSpeed(leftWheel, newLeftVel);
                SetSpeed(rightWheel, newRightVel);

                if (Mathf.Approximately(newLeftVel, 0f) && Mathf.Approximately(newRightVel, 0f))
                {
                    Debug.LogWarning("[Safety] Robot fermato dolcemente.");
                    isStopped = true;
                }
            }
        }
    }

    void LeftWheelCmdCallback(Float64Msg msg)
    {
        lastCmdReceivedTime = Time.time;
        isStopped = false;
        SetSpeed(leftWheel, (float)msg.data);
    }

    void RightWheelCmdCallback(Float64Msg msg)
    {
        lastCmdReceivedTime = Time.time;
        isStopped = false;
        SetSpeed(rightWheel, (float)msg.data);
    }

    // Funzione helper per applicare i parametri di forza senza cambiare velocità
    void ApplyDriveParameters(ArticulationBody wheel)
    {
        if (wheel == null) return;
        var drive = wheel.xDrive;
        drive.forceLimit = wheelForceLimit;
        drive.damping = wheelDamping;
        drive.stiffness = wheelStiffness;
        wheel.xDrive = drive;
    }

    void SetSpeed(ArticulationBody wheel, float speedRadS)
    {
        if (wheel == null) return;

        var drive = wheel.xDrive;

        // 1. Applichiamo SEMPRE i parametri fisici per assicurarci che siano aggiornati dall'Inspector
        drive.forceLimit = wheelForceLimit;
        drive.damping = wheelDamping;
        drive.stiffness = wheelStiffness;

        // 2. Impostiamo la velocità
        float targetDeg = speedRadS * Mathf.Rad2Deg;

        // Piccolo check per evitare jittering inutile, ma aggiorniamo comunque se i parametri fisici sono cambiati
        drive.targetVelocity = targetDeg;

        wheel.xDrive = drive;
    }

    void PublishWheelStates()
    {
        JointStateMsg stateMsg = new JointStateMsg();
        stateMsg.name = new string[] { "right_wheel_joint", "left_wheel_joint" };

        stateMsg.velocity = new double[] {
            rightWheel.jointVelocity[0],
            leftWheel.jointVelocity[0]
        };

        ros.Publish(wheelsStateTopic, stateMsg);
    }
}