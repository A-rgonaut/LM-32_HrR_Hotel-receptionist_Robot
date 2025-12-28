using UnityEngine;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Per il messaggio String

public class CharacterSelector : MonoBehaviour
{
    ROSConnection ros;
    [System.Serializable]
    public class CharacterData
    {
        public int id;
        public string nome;
        public string cognome;
        public GameObject cam;
    }

    public CharacterData[] characters;

    // NUOVO: Questa variabile memorizza chi � attivo in questo momento
    private int _currentCharacterIndex = 0;
    public string topicName = "bottone"; // Il nome del topic ROS 2

    void Start()
    {
        ActivateCharacter(0);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    public void ActivateCharacter(int indexToActivate)
    {
        // NUOVO: Aggiorniamo l'indice corrente
        _currentCharacterIndex = indexToActivate;

        for (int i = 0; i < characters.Length; i++)
        {
            if (i == indexToActivate)
            {
                characters[i].cam.SetActive(true);
                // Print di conferma cambio personaggio

                Debug.Log($"Selezionato -> ID: {characters[i].id} | Nome: {characters[i].nome} | Cognome: {characters[i].cognome}");
            }
            else
            {
                characters[i].cam.SetActive(false);
            }
        }
    }

    // NUOVO: Questa � la funzione per il tuo NUOVO pulsante d'azione
    public void EseguiAzionePersonaggio(GameObject buttonClicked)
    {
        // Recuperiamo i dati del personaggio attivo usando l'indice salvato
        CharacterData attivo = characters[_currentCharacterIndex];
        TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();

        // Facciamo la print richiesta
        StringMsg msg = new StringMsg( $"Utente {attivo.id} {attivo.nome} {attivo.cognome} ha avviato lo scenario {testoComponent.text}");
        
        ros.Publish(topicName, msg);
        Debug.Log(msg);
    }
}