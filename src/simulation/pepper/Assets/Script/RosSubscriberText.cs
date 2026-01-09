using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using TMPro; // Necessario per Text Mesh Pro
using RosMessageTypes.Std; // Assicurati di aver generato i messaggi Std

public class RosSubscriberText : MonoBehaviour
{
    public string topicName = "/unity/dialogo_robot"; // Il nome del topic ROS 2
    public TextMeshProUGUI textDisplay;   // Trascina qui il tuo oggetto Text (TMP)

    void Start()
    {
        // Ci iscriviamo al topic.
        // Specifichiamo <StringMsg> come tipo di messaggio (std_msgs/String)
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(topicName, UpdateText);
    }

    void UpdateText(StringMsg msg)
    {
        // Questa funzione viene chiamata ogni volta che arriva un messaggio
        // AGGIORNAMENTO: Unity richiede che le modifiche alla UI avvengano sul thread principale
        textDisplay.text += "Messaggio: " + msg.data + "\n";

        Debug.Log("Ricevuto da ROS 2: " + msg.data);
    }
}
