using RosMessageTypes.Std;
using System.Collections;
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

    public CharacterData[] characters;

    [Header("UI Control")]
    public GameObject[] actionButtons;
    public TextMeshProUGUI bpmText;
    public TextMeshProUGUI pasText;
    public TextMeshProUGUI padText;

    [Header("Simulation Settings")]
    [Range(0, 20)]
    public int noiseRange = 5;

    private int _currentCharacterIndex = 0;
    public string buttonTopicName = "bottone";
    public string healthTopicName = "health_raw";

    [Header("Comandi Manuali")]
    public Transform WaypointScenarioA;
    public Transform WaypointScenarioB;

    private Coroutine vitalsCoroutine;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(buttonTopicName);
        ros.RegisterPublisher<StringMsg>(healthTopicName);
        ActivateCharacter(0);
    }

    // Gestione GUI per il personaggio attivo
    public void ActivateCharacter(int indexToActivate)
    {
        _currentCharacterIndex = indexToActivate;

        // 1. GESTIONE CAMERE
        for (int i = 0; i < characters.Length; i++)
        {
            characters[i].cam.SetActive(i == indexToActivate);
        }

        // 2. GESTIONE PULSANTI (Nascondi se ID = 0)
        int currentId = characters[indexToActivate].id;
        bool showButtons = (currentId != 0);
        foreach (GameObject btn in actionButtons)
        {
            if (btn != null) btn.SetActive(showButtons);
        }

        // 3. START LOOP VITALI
        if (vitalsCoroutine != null) StopCoroutine(vitalsCoroutine);
        vitalsCoroutine = StartCoroutine(SimulateAndPublishVitals());
    }

    IEnumerator SimulateAndPublishVitals()
    {
        CultureInfo culture = CultureInfo.InvariantCulture;

        while (true)
        {
            CharacterData data = characters[_currentCharacterIndex];

            // --- MODIFICA RICHIESTA ---
            // Se l'ID � 0, non calcolare nulla e non inviare nulla.
            if (data.id == 0)
            {
                // Opzionale: Mostriamo dei trattini per indicare che non ci sono dati
                if (bpmText != null) bpmText.text = "---";
                if (pasText != null) pasText.text = "---";
                if (padText != null) padText.text = "---";

                // Aspetta 1 secondo e ricomincia il ciclo (saltando il codice sotto)
                yield return new WaitForSeconds(1.0f);
                continue;
            }
            // --------------------------

            // 1. Calcolo Vitali
            int currentBPM = data.avgBPM + Random.Range(-noiseRange, noiseRange + 1);
            int currentPAS = data.avgPAS + Random.Range(-noiseRange, noiseRange + 1);
            int currentPAD = data.avgPAD + Random.Range(-noiseRange, noiseRange + 1);

            // 2. Recupero Posizione
            Transform target = data.bodyTransform != null ? data.bodyTransform : data.cam.transform;
            Vector3 pos = target.position;

            // 3. UI Update
            if (bpmText != null) bpmText.text = "BPM: " + currentBPM.ToString();
            if (pasText != null) pasText.text = "PAS: " + currentPAS.ToString();
            if (padText != null) padText.text = "PAD: " + currentPAD.ToString();

            // 4. JSON
            string json = $@"{{
                ""id"": {data.id},
                ""hr"": {currentBPM},
                ""pmax"": {currentPAS},
                ""pmin"": {currentPAD},
                ""x"": {pos.x.ToString("F2", culture)},
                ""y"": {pos.y.ToString("F2", culture)},
                ""z"": {pos.z.ToString("F2", culture)}
            }}";

            json = json.Replace("\n", "").Replace("\r", "").Replace(" ", "");

            //Debug.Log(json);

            // 5. Invio ROS
            ros.Publish(healthTopicName, new StringMsg(json));

            yield return new WaitForSeconds(1.0f);
        }
    }

    public void StartScenarioA()
    {
        // 1. Recupera i dati del personaggio attualmente selezionato
        CharacterData activeChar = characters[_currentCharacterIndex];

        // 2. Controllo di sicurezza (se è l'ID 0 che è "muto", magari non vogliamo muoverlo)
        if (activeChar.id == 0)
        {
            Debug.Log("L'utente ID 0 non può essere spostato.");
            return;
        }

        // 3. Trova lo script NPCController sul personaggio.
        // Usiamo 'bodyTransform' se l'hai impostato, altrimenti cerchiamo sull'oggetto della camera o sui genitori
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;

        // Cerchiamo il componente NPCController (potrebbe essere sul padre o sull'oggetto stesso)
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        // 4. Esegui il comando
        if (controller != null)
        {
            Debug.Log($"{activeChar.nome} ha avviato lo scenario A");
            controller.GoToTargetAndStay(WaypointScenarioA);
        }
        else
        {
            Debug.LogError("Non ho trovato lo script NPCController sul personaggio attivo!");
        }
    }

    public void StartScenarioB()
    {
        // 1. Recupera i dati del personaggio attualmente selezionato
        CharacterData activeChar = characters[_currentCharacterIndex];

        // 2. Controllo di sicurezza (se è l'ID 0 che è "muto", magari non vogliamo muoverlo)
        if (activeChar.id == 0)
        {
            Debug.Log("L'utente ID 0 non può essere spostato.");
            return;
        }

        // 3. Trova lo script NPCController sul personaggio.
        // Usiamo 'bodyTransform' se l'hai impostato, altrimenti cerchiamo sull'oggetto della camera o sui genitori
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;

        // Cerchiamo il componente NPCController (potrebbe essere sul padre o sull'oggetto stesso)
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        // 4. Esegui il comando
        if (controller != null)
        {
            Debug.Log($"{activeChar.nome} ha avviato lo scenario B");
            controller.GoToTargetAndStay(WaypointScenarioB);
        }
        else
        {
            Debug.LogError("Non ho trovato lo script NPCController sul personaggio attivo!");
        }

        // TODO: implementare logica che invia l'allarme ROS a Pepper
    }

    public void StartScenarioC()
    {
        // 1. Recupera il personaggio attivo
        CharacterData activeChar = characters[_currentCharacterIndex];

        // 2. Controllo sicurezza ID 0
        if (activeChar.id == 0)
        {
            Debug.Log("L'utente ID 0 non può avere un malore.");
            return;
        }

        // 3. Trova il controller
        Transform targetObj = activeChar.bodyTransform != null ? activeChar.bodyTransform : activeChar.cam.transform;
        NPCController controller = targetObj.GetComponent<NPCController>();
        if (controller == null) controller = targetObj.GetComponentInParent<NPCController>();

        // 4. Esegui il comando di caduta
        if (controller != null)
        {
            Debug.Log($"{activeChar.nome} sta avendo un malore (Scenario C)");
            controller.PerformDying();

            // TODO: Qui potresti voler inviare un messaggio ROS specifico di allarme
            // Esempio: Inviare un messaggio String o Bool su un topic "/health_alarm"
        }
        else
        {
            Debug.LogError("Controller non trovato per Scenario C!");
        }
    }

    public void EseguiAzionePersonaggio()
    {
        // Chiediamo a Unity: "Chi è che è stato appena cliccato?"
        GameObject buttonClicked = EventSystem.current.currentSelectedGameObject;

        if (buttonClicked == null) return;

        CharacterData attivo = characters[_currentCharacterIndex];

        // Per sicurezza controlliamo che l'ID non sia 0
        if (attivo.id == 0) return;

        TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();

        if (testoComponent != null)
        {
            StringMsg msg = new StringMsg($"{{\"id\":{attivo.id},\"nome\":\"{attivo.nome}\",\"cognome\":\"{attivo.cognome}\",\"bottone\":\"{testoComponent.text}\"}}");
            Debug.Log("Invio bottone: " + msg.data);
            ros.Publish(buttonTopicName, msg);
        }
    }
}
