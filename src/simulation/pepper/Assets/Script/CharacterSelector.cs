using UnityEngine;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections;
using System.Globalization;

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

    private Coroutine vitalsCoroutine;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(buttonTopicName);
        ros.RegisterPublisher<StringMsg>(healthTopicName);
        ActivateCharacter(0);
    }

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

    public void EseguiAzionePersonaggio(GameObject buttonClicked)
    {
        CharacterData attivo = characters[_currentCharacterIndex];
        // Per sicurezza, impediamo l'invio anche dal pulsante se ID=0 (nel caso i bottoni fossero rimasti attivi per errore)
        //if (attivo.id == 0) return;

        TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();
        StringMsg msg = new StringMsg($"{{\"id\":{0},\"nome\":\"{attivo.nome}\",\"cognome\":\"{attivo.cognome}\",\"bottone\":\"{testoComponent.text}\"}}");
        ros.Publish(buttonTopicName, msg);
    }
}