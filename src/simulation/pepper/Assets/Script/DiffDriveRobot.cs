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

    [Header("Safety Watchdog")]
    public float commandTimeout = 0.5f; // Tempo massimo senza comandi prima dello stop
    private float lastCmdReceivedTime;
    private bool isStopped = false; // Per evitare di settare 0 continuamente

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

        // Inizializza il timer
        lastCmdReceivedTime = Time.time;

        Debug.Log("DiffDriveRobot avviato. In attesa di comandi ROS...");
    }

    void FixedUpdate()
    {
        // 1. Controllo di Sicurezza (Watchdog)
        CheckConnectionWatchdog();

        // 2. Pubblicazione Stato Ruote
        timeElapsed += Time.fixedDeltaTime;
        if (timeElapsed >= 1.0f / publishRate)
        {
            PublishWheelStates();
            timeElapsed = 0;
        }
    }

    // --- NUOVO METODO DI SICUREZZA ---
    void CheckConnectionWatchdog()
    {
        // Se è passato troppo tempo dall'ultimo comando ricevuto
        if (Time.time - lastCmdReceivedTime > commandTimeout)
        {
            // Se non siamo già fermi, fermiamo tutto
            if (!isStopped)
            {
                Debug.LogWarning($"[Safety] Nessun comando ricevuto per {commandTimeout}s. STOP DI EMERGENZA.");
                SetSpeed(leftWheel, 0);
                SetSpeed(rightWheel, 0);
                isStopped = true;
            }
        }
    }

    void LeftWheelCmdCallback(Float64Msg msg)
    {
        // Resetta il timer del watchdog
        lastCmdReceivedTime = Time.time;
        isStopped = false;

        // LOG: Vediamo se arriva il comando
        // Debug.Log($"[ROS -> Unity] CMD Sinistra: {msg.data} rad/s");
        SetSpeed(leftWheel, (float)msg.data);
    }

    void RightWheelCmdCallback(Float64Msg msg)
    {
        // Resetta il timer del watchdog
        lastCmdReceivedTime = Time.time;
        isStopped = false;

        // LOG: Vediamo se arriva il comando
        // Debug.Log($"[ROS -> Unity] CMD Destra: {msg.data} rad/s");
        SetSpeed(rightWheel, (float)msg.data);
    }

    void SetSpeed(ArticulationBody wheel, float speedRadS)
    {
        // NOTA BENE: Qui assumiamo che la ruota giri sull'asse X (xDrive).
        var drive = wheel.xDrive;
        drive.targetVelocity = speedRadS * Mathf.Rad2Deg; // Conversione rad/s -> deg/s
        wheel.xDrive = drive;
    }

    void PublishWheelStates()
    {
        JointStateMsg stateMsg = new JointStateMsg();

        // I nomi devono corrispondere a quelli definiti nel URDF/ROS
        stateMsg.name = new string[] { "right_wheel_joint", "left_wheel_joint" }; // Assicurati che i nomi siano corretti

        // Prende la velocità attuale (in rad/s, Unity la fornisce già convertita se configurato bene)
        double velRight = rightWheel.jointVelocity[0];
        double velLeft = leftWheel.jointVelocity[0];

        stateMsg.velocity = new double[] { velRight, velLeft };

        ros.Publish(wheelsStateTopic, stateMsg);
    }
}