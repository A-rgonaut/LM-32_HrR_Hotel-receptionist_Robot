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
    public float commandTimeout = 0.5f; // Tempo massimo senza comandi
    
    [Tooltip("Decelerazione in rad/s^2. Valori bassi = frenata dolce. Valori alti = frenata brusca.")]
    public float brakingDeceleration = 2.0f; // NUOVO: Forza della frenata

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

        lastCmdReceivedTime = Time.time;
        Debug.Log("DiffDriveRobot avviato.");
    }
    void OnDestroy()
    {
        ros.Disconnect();
    }

    void FixedUpdate()
    {
        // 1. Controllo Watchdog e Frenata
        CheckConnectionWatchdog();

        // 2. Pubblicazione Stato
        timeElapsed += Time.fixedDeltaTime;
        if (timeElapsed >= 1.0f / publishRate)
        {
            PublishWheelStates();
            timeElapsed = 0;
        }
    }

    void CheckConnectionWatchdog()
    {
        // Se è scaduto il tempo limite...
        if (Time.time - lastCmdReceivedTime > commandTimeout)
        {
            // ... e se il robot non è ancora completamente fermo
            if (!isStopped)
            {
                // LOGICA DI FRENATA PROGRESSIVA
                
                // 1. Recuperiamo la velocità target attuale (convertendo da deg a rad per coerenza)
                float currentLeftVel = leftWheel.xDrive.targetVelocity * Mathf.Deg2Rad;
                float currentRightVel = rightWheel.xDrive.targetVelocity * Mathf.Deg2Rad;

                // 2. Calcoliamo quanto ridurre la velocità in questo frame
                float step = brakingDeceleration * Time.fixedDeltaTime;

                // 3. Ci avviciniamo a 0 gradualmente
                float newLeftVel = Mathf.MoveTowards(currentLeftVel, 0f, step);
                float newRightVel = Mathf.MoveTowards(currentRightVel, 0f, step);

                // 4. Applichiamo la nuova velocità ridotta
                SetSpeed(leftWheel, newLeftVel);
                SetSpeed(rightWheel, newRightVel);

                // 5. Se entrambe sono praticamente a 0, dichiariamo lo stop completo
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

    void SetSpeed(ArticulationBody wheel, float speedRadS)
    {
        if (wheel == null) return;

        var drive = wheel.xDrive;
        // Convertiamo senza troncare per mantenere la massima fedeltà fisica
        float targetDeg = speedRadS * Mathf.Rad2Deg;

        // Evita di aggiornare se la differenza è infinitesimale (ottimizzazione)
        if (Mathf.Abs(drive.targetVelocity - targetDeg) > 0.0001f)
        {
            drive.targetVelocity = targetDeg;
            wheel.xDrive = drive;
        }
    }

    void PublishWheelStates()
    {
        JointStateMsg stateMsg = new JointStateMsg();
        stateMsg.name = new string[] { "right_wheel_joint", "left_wheel_joint" }; // Verifica i nomi nel tuo URDF
        
        stateMsg.velocity = new double[] { 
            rightWheel.jointVelocity[0], 
            leftWheel.jointVelocity[0] 
        };

        //Debug.Log($"Right: {rightWheel.jointVelocity[0]}, Left: {leftWheel.jointVelocity[0]}");

        ros.Publish(wheelsStateTopic, stateMsg);
    }
}