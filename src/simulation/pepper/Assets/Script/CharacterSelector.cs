using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.EventSystems;

public class CharacterSelector : MonoBehaviour
{
    ROSConnection ros;

    [System.Serializable]

    public class CharacterData
    {
        public int id;
        public string nome;
        public string cognome;

        [Header("Riferimenti Scena")]
        public GameObject cam;
        public Transform bodyTransform;

        [Header("Parametri Vitali (Media)")]
        public int avgBPM = 75;
        public int avgPAS = 120;
        public int avgPAD = 80;

        [Header("Comandi Manuali")]
        public Transform emergencyPoint;
    }

    [Header("Database")]
    public Neo4jManager dbManager;

    [Header("Pepper")]
    public GameObject pepperCamera;
    public RobotBattery pepperBatteryScript;

    [Header("Characters")]
    public CharacterData[] characters;

    [Header("UI Control")]
    public GameObject[] actionButtons;
    public TextMeshProUGUI bpmText;
    public TextMeshProUGUI pasText;
    public TextMeshProUGUI padText;

    [Header("Simulation Settings")]
    [Range(0, 20)]
    public int noiseRange = 5;

    private int _currentCharacterIndex = -1;
    public string buttonTopicName = "/unity/bottone";
    public string healthTopicName = "/unity/health_raw";

    [Header("Comandi Manuali")]
    public Transform WaypointScenarioA;
    public Transform WaypointScenarioB;

    private Coroutine vitalsCoroutine;
    async void Start()
    {
        // 1. Setup ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(buttonTopicName);
        ros.RegisterPublisher<StringMsg>(healthTopicName);

        // 2. RECUPERO DATI DA NEO4J
        if (dbManager != null)
        {
            Debug.Log("Connessione a Neo4j in corso...");

            // Attendiamo i dati
            var dbData = await dbManager.GetCharactersFromDB();

            if (dbData.Count > 0)
            {
                // 3. Sovrascriviamo i dati nell'array esistente
                for (int i = 0; i < characters.Length; i++)
                {
                    // Controllo di sicurezza per non andare fuori indice se il DB ha meno righe
                    if (i < dbData.Count)
                    {
                        characters[i].id = dbData[i].id;
                        characters[i].nome = dbData[i].nome;
                        characters[i].cognome = dbData[i].cognome;

                        Debug.Log($"Caricato da DB: {characters[i].nome} su slot {i}");
                    }
                }
            }
            else
            {
                Debug.LogWarning("Nessun dato trovato nel DB o connessione fallita. Uso dati Inspector.");
            }
        }
        else
        {
            Debug.LogError("Neo4jManager non assegnato nell'Inspector!");
        }

        vitalsCoroutine = StartCoroutine(SimulateAndPublishVitals());

        // 4. Attiva il primo personaggio SOLO dopo aver caricato i dati
        ActivateCharacter(0);
    }

    void Update()
    {
        if ((Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)) &&
            Input.GetKey(KeyCode.T) &&
            Input.GetKeyDown(KeyCode.V))
        {
            TriggerEasterEgg();
        }
    }
    void OnDestroy()
    {
        ros.Disconnect();
    }

    public void ActivatePepper()
    {
        _currentCharacterIndex = -1;

        // 1. Attiva Camera
        if (pepperCamera != null) pepperCamera.SetActive(true);

        // 2. Attiva BATTERIA e UI Robot
        if (pepperBatteryScript != null)
        {
            pepperBatteryScript.enabled = true; // Riattiva la logica
            if (pepperBatteryScript.batteryUIPanel != null)
                pepperBatteryScript.batteryUIPanel.SetActive(true); // Mostra la UI
        }

        // 3. Disattiva Camera NPC
        for (int i = 0; i < characters.Length; i++)
            if (characters[i].cam != null) characters[i].cam.SetActive(false);

        // 4. Nascondi UI Umani (Bottoni scenari)
        foreach (GameObject btn in actionButtons)
            if (btn != null) btn.SetActive(false);

        // 5. Pulisci la UI dei parametri vitali
        if (bpmText != null) bpmText.text = "---";
        if (pasText != null) pasText.text = "---";
        if (padText != null) padText.text = "---";

        Debug.Log("Attivato Pepper (Modalità Osservatore)");
    }

    // Gestione GUI per il personaggio attivo
    public void ActivateCharacter(int indexToActivate)
    {
        _currentCharacterIndex = indexToActivate;

        // 1. Spegni Camera Pepper
        if (pepperCamera != null) pepperCamera.SetActive(false);

        // 2. DISATTIVA BATTERIA e UI Robot
        if (pepperBatteryScript != null)
        {
            pepperBatteryScript.enabled = true; // Ferma il calcolo (opzionale, o lascia true se vuoi che si scarichi anche mentre non lo guardi)
            if (pepperBatteryScript.batteryUIPanel != null)
                pepperBatteryScript.batteryUIPanel.SetActive(false); // Nasconde la UI
        }

        // 3. Gestione Camere NPC (Accendi solo quella scelta)
        for (int i = 0; i < characters.Length; i++)
            if (characters[i].cam != null)
                characters[i].cam.SetActive(i == indexToActivate);
        
        // 4. Mostra i pulsanti (perché siamo su un NPC)
        foreach (GameObject btn in actionButtons)
            if (btn != null) btn.SetActive(true);

        Debug.Log($"Attivato NPC: {characters[indexToActivate].nome}");
    }

    public CharacterData GetActiveCharacter()
    {
        return characters[_currentCharacterIndex];
    }

    IEnumerator SimulateAndPublishVitals()
    {
        CultureInfo culture = CultureInfo.InvariantCulture;

        while (true)
        {
            List<string> allCharactersJson = new List<string>();

            for (int i = 0; i < characters.Length; i++)
            {
                CharacterData data = characters[i];
                Transform target = data.bodyTransform != null ? data.bodyTransform : data.cam.transform;

                //Recupera lo stato dall'NPCController 
                bool isCritical = false;
                NPCController npcCtrl = target.GetComponent<NPCController>();
                if (npcCtrl == null) npcCtrl = target.GetComponentInParent<NPCController>();

                if (npcCtrl != null) isCritical = npcCtrl.isCriticalCondition;

                int currentBPM = 0, currentPAS = 0, currentPAD = 0;

                if (isCritical)
                {
                    // Scenario C2: Morto -> Tutto a zero
                    currentBPM = 0;
                    currentPAS = 0;
                    currentPAD = 0;
                }
                else
                {
                    // Normale
                    currentBPM = data.avgBPM + Random.Range(-noiseRange, noiseRange + 1);
                    currentPAS = data.avgPAS + Random.Range(-noiseRange, noiseRange + 1);
                    currentPAD = data.avgPAD + Random.Range(-noiseRange, noiseRange + 1);
                }

                // ... (Recupero posizione e update UI rimangono uguali) ...
                Vector3 pos = target.position;

                if (i == _currentCharacterIndex)
                {
                    if (bpmText != null) bpmText.text = "BPM: " + currentBPM.ToString();
                    if (pasText != null) pasText.text = "PAS: " + currentPAS.ToString();
                    if (padText != null) padText.text = "PAD: " + currentPAD.ToString();
                }

                // ... (Creazione JSON e loop continuano uguali) ...
                string charJson = $@"{{
                    ""id"": {data.id},
                    ""hr"": {currentBPM},
                    ""pmax"": {currentPAS},
                    ""pmin"": {currentPAD},
                    ""x"": {pos.x.ToString("F2", culture)},
                    ""y"": {pos.y.ToString("F2", culture)},
                    ""z"": {pos.z.ToString("F2", culture)}
                }}";

                charJson = charJson.Replace("\n", "").Replace("\r", "").Replace(" ", "");
                allCharactersJson.Add(charJson);
            }

            string finalJsonArray = "[" + string.Join(",", allCharactersJson) + "]";
            ros.Publish(healthTopicName, new StringMsg(finalJsonArray));

            yield return new WaitForSeconds(1.0f);
        }
    }

    public void sendToRos2(TextMeshProUGUI testoComponent, CharacterData activeChar)
    {
        if (testoComponent != null)
        {
            StringMsg msg = new StringMsg($"{{\"id\":{activeChar.id},\"nome\":\"{activeChar.nome}\",\"cognome\":\"{activeChar.cognome}\",\"bottone\":\"{testoComponent.text}\"}}");
            Debug.Log("Invio bottone: " + msg.data);
            ros.Publish(buttonTopicName, msg);
        }
    }

    public void StartScenarioA()
    {
        // 1. Recupera i dati del personaggio attualmente selezionato
        CharacterData activeChar = characters[_currentCharacterIndex];

        // 2. Trova lo script NPCController sul personaggio.
        // Usiamo 'bodyTransform' se l'hai impostato, altrimenti cerchiamo sull'oggetto della camera o sui genitori
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;

        // Cerchiamo il componente NPCController (potrebbe essere sul padre o sull'oggetto stesso)
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        // 3. Esegui il comando
        if (controller != null){
            Debug.Log($"{activeChar.nome} ha avviato lo scenario A");
            controller.GoToTargetAndStay(WaypointScenarioA);
        }else{
            Debug.LogError("Non ho trovato lo script NPCController sul personaggio attivo!");
        }

        // Chiediamo a Unity: "Chi è che è stato appena cliccato?"
        GameObject buttonClicked = EventSystem.current.currentSelectedGameObject;

        if (buttonClicked == null) return;

        TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();

        sendToRos2(testoComponent, activeChar);
    }

    public void StartScenarioB()
    {
        // 1. Recupera i dati del personaggio attualmente selezionato
        CharacterData activeChar = characters[_currentCharacterIndex];

        // 2. Trova lo script NPCController sul personaggio.
        // Usiamo 'bodyTransform' se l'hai impostato, altrimenti cerchiamo sull'oggetto della camera o sui genitori
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;

        // Cerchiamo il componente NPCController (potrebbe essere sul padre o sull'oggetto stesso)
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        // 3. Esegui il comando
        if (controller != null){
            Debug.Log($"{activeChar.nome} ha avviato lo scenario B");
            controller.GoToTargetAndStay(WaypointScenarioB);
        }else{
            Debug.LogError("Non ho trovato lo script NPCController sul personaggio attivo!");
        }

        // Chiediamo a Unity: "Chi è che è stato appena cliccato?"
        GameObject buttonClicked = EventSystem.current.currentSelectedGameObject;

        if (buttonClicked == null) return;

        TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();

        sendToRos2(testoComponent, activeChar);

        // TODO: implementare logica che invia l'allarme ROS a Pepper
    }

    public void StartScenarioC()
    {
        CharacterData activeChar = characters[_currentCharacterIndex];

        // Trova Controller (come facevi prima)
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        if (controller != null)
        {
            Debug.Log($"{activeChar.nome} Scenario C: Inginocchiamento");
            // Chiama la nuova funzione specifica
            controller.PerformKneeling();
        }
        else
        {
            Debug.LogError("Controller non trovato!");
        }

        // Gestione bottone e invio ROS
        GameObject buttonClicked = EventSystem.current.currentSelectedGameObject;
        if (buttonClicked != null)
        {
            TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();
            sendToRos2(testoComponent, activeChar);
        }
    }

    public void StartScenarioC2()
    {
        CharacterData activeChar = characters[_currentCharacterIndex];

        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        if (controller != null)
        {
            Debug.Log($"{activeChar.nome} Scenario C2: Malore Critico (Parametri 0)");
            // Chiama la funzione di morte che setterà anche isCriticalCondition = true
            controller.PerformDying();
        }
        else
        {
            Debug.LogError("Controller non trovato!");
        }

        // Gestione bottone e invio ROS
        GameObject buttonClicked = EventSystem.current.currentSelectedGameObject;
        if (buttonClicked != null)
        {
            TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();
            sendToRos2(testoComponent, activeChar);
        }
    }
    void TriggerEasterEgg()
    {
        if (_currentCharacterIndex == -1) return;

        CharacterData activeChar = characters[_currentCharacterIndex];

        // Recupera il riferimento corretto (come fai negli altri metodi)
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        if (controller != null) controller.ToggleDance();
    }
}
