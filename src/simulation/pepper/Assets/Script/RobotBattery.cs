using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Usiamo StringMsg per inviare il JSON

public class RobotBattery : MonoBehaviour
{
    [Header("Impostazioni Batteria")]
    [Range(0, 100)] public float currentBattery = 100f;
    public float dischargeRate = 2.0f;
    public float chargeRate = 10.0f;

    [Header("Riferimenti UI")]
    public GameObject batteryUIPanel;
    public Slider batterySlider;
    public TextMeshProUGUI batteryText;

    [Header("ROS Settings")]
    public string batteryTopic = "/unity/battery_state";
    public float publishRate = 1.0f;

    [Header("Debug")]
    public bool canRecharge = true;
    public bool isCharging = false;

    private ROSConnection ros;
    private float timeElapsed = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(batteryTopic);
    }

    void Update()
    {
        // 1. Gestione Carica/Scarica
        if (isCharging && canRecharge)
            currentBattery += chargeRate * Time.deltaTime;
        else
            currentBattery -= dischargeRate * Time.deltaTime;

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
        if (batterySlider != null) batterySlider.value = currentBattery / 100f;
        if (batteryText != null) batteryText.text = $"Bat: {Mathf.RoundToInt(currentBattery)}%";

        if (batteryText != null && currentBattery < 20) batteryText.color = Color.red;
        else if (batteryText != null) batteryText.color = Color.white;
    }

    // MODIFICA: Funzione aggiornata per inviare JSON
    void PublishBatteryState()
    {
        int batteryInt = Mathf.RoundToInt(currentBattery);
        
        // Creiamo il JSON manualmente.
        // Esempio output: {"level": 85, "is_charging": true}
        // Nota: convertiamo il booleano isCharging in "true" o "false" minuscoli per compatibilità JSON standard
        string jsonString = $"{{\"level\": {batteryInt}, \"is_charging\": {isCharging.ToString().ToLower()}}}";

        StringMsg msg = new StringMsg(jsonString);
        ros.Publish(batteryTopic, msg);
    }

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