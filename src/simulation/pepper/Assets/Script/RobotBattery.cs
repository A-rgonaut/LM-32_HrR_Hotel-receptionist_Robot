using UnityEngine;
using UnityEngine.UI; // Se usi Slider standard
using TMPro;          // Se usi TextMeshPro
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // O sensor_msgs/BatteryState se preferisci, qui uso Int32 o String per semplicità

public class RobotBattery : MonoBehaviour
{
    [Header("Impostazioni Batteria")]
    [Range(0, 100)] public float currentBattery = 100f;
    public float dischargeRate = 2.0f; // Quanto scarica al secondo
    public float chargeRate = 10.0f;   // Quanto carica al secondo

    [Header("Riferimenti UI")]
    public GameObject batteryUIPanel; // Il pannello che contiene slider/testo
    public Slider batterySlider;      // Opzionale
    public TextMeshProUGUI batteryText; // Opzionale

    [Header("ROS Settings")]
    public string batteryTopic = "/unity/battery_state";
    public float publishRate = 1.0f; // Ogni quanto inviare i dati

    [Header("Debug")]
    public bool canRecharge = true; // Funzionalità attivabile/disattivabile
    public bool isCharging = false;

    private ROSConnection ros;
    private float timeElapsed = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Usiamo Int32Msg per inviare la percentuale intera (0-100)
        ros.RegisterPublisher<Int32Msg>(batteryTopic);
    }

    void Update()
    {
        // 1. Gestione Carica/Scarica
        if (isCharging && canRecharge)
        {
            currentBattery += chargeRate * Time.deltaTime;
        }
        else
        {
            currentBattery -= dischargeRate * Time.deltaTime;
        }

        // Clamp per restare tra 0 e 100
        currentBattery = Mathf.Clamp(currentBattery, 0f, 100f);

        // 2. Aggiornamento UI
        UpdateUI();

        // 3. Invio ROS temporizzato
        timeElapsed += Time.deltaTime;
        if (timeElapsed >= publishRate)
        {
            PublishBatteryState();
            timeElapsed = 0;
        }
    }

    void UpdateUI()
    {
        if (batterySlider != null) batterySlider.value = currentBattery / 100f; // Slider vuole 0-1
        if (batteryText != null) batteryText.text = $"Bat: {Mathf.RoundToInt(currentBattery)}%";

        // Cambio colore se scarica (opzionale)
        if (batteryText != null && currentBattery < 20) batteryText.color = Color.red;
        else if (batteryText != null) batteryText.color = Color.white;
    }

    void PublishBatteryState()
    {
        // Inviamo un intero
        Int32Msg msg = new Int32Msg((int)currentBattery);
        ros.Publish(batteryTopic, msg);
    }

    // --- Rilevamento Stazione di Ricarica ---

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("ChargingStation"))
        {
            isCharging = true;
            Debug.Log("Robot in carica...");
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("ChargingStation"))
        {
            isCharging = false;
            Debug.Log("Robot scollegato dalla carica.");
        }
    }
}