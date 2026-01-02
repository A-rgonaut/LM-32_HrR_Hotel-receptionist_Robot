using UnityEngine;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;  // Messaggio String

public class CharacterSelector : MonoBehaviour {
    ROSConnection ros;
    [System.Serializable]
    public class CharacterData {
        public int id;
        public string nome;
        public string cognome;
        public GameObject cam;
    }

    public CharacterData[] characters;

    private int _currentCharacterIndex = 0;  // chi e' attivo in questo momento
    public string topicName = "bottone";     // nome topic ROS 2

    void Start() {
        ActivateCharacter(0);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    public void ActivateCharacter(int indexToActivate) {
        _currentCharacterIndex = indexToActivate;  // Aggiorniamo l'indice corrente
        for (int i = 0; i < characters.Length; i++) {
            if (i == indexToActivate) {
                characters[i].cam.SetActive(true);
                Debug.Log($"Selezionato -> ID: {characters[i].id} | Nome: {characters[i].nome} | Cognome: {characters[i].cognome}");
            } else {
                characters[i].cam.SetActive(false);
            }
        }
    }

    // funzione per il tuo NUOVO pulsante d'azione
    public void EseguiAzionePersonaggio(GameObject buttonClicked) {
        // Recuperiamo i dati del personaggio attivo usando l'indice salvato
        CharacterData attivo = characters[_currentCharacterIndex];
        TextMeshProUGUI testoComponent = buttonClicked.GetComponentInChildren<TextMeshProUGUI>();
        // StringMsg msg = new StringMsg($"{{\"id\": {attivo.id}, \"nome\": \"{attivo.nome}\", \"cognome\": \"{attivo.cognome}\", \"bottone\": \"{testoComponent.text}\"}}");
        StringMsg msg = new StringMsg($"{{\"id\": 0, \"nome\": \"{attivo.nome}\", \"cognome\": \"{attivo.cognome}\", \"bottone\": \"{testoComponent.text}\"}}");
        Debug.Log(msg);
        ros.Publish(topicName, msg);
    }
}
