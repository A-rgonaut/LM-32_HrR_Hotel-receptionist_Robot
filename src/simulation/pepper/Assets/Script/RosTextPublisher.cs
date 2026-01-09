using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro; // 1. NECESSARIO se usi TextMeshPro (altrimenti usa UnityEngine.UI)

public class RosTextPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity/dialogo_umano";

    // 2. Aggiungi questo riferimento per poter controllare la casella di testo
    public TMP_InputField inputFieldToClear;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }
    void OnDestroy()
    {
        ros.Disconnect();
    }

    public void PublishMessage(string messageContent)
    {
        // Controllo di sicurezza: se il messaggio è vuoto, non fare nulla
        if (string.IsNullOrWhiteSpace(messageContent)) return;

        // Logica ROS originale
        StringMsg msg = new StringMsg(messageContent);
        ros.Publish(topicName, msg);
        Debug.Log("Inviato a ROS: " + messageContent);

        // 3. Cancella il testo usando il riferimento
        if (inputFieldToClear != null)
        {
            inputFieldToClear.text = "";

            // Opzionale: mantiene il cursore attivo per scrivere subito il prossimo messaggio
            inputFieldToClear.ActivateInputField();
        }
    }
}
