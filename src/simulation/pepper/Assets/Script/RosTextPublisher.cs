using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro;

public class RosTextPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity/dialogo_umano";

    public TMP_InputField inputFieldToClear;

    [Header("Collegamenti")]
    public CharacterSelector characterSelector; 
    public RosSubscriberText chatDisplay;       

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    public void PublishMessage(string messageContent)
    {
        if (string.IsNullOrWhiteSpace(messageContent)) return;

        // 1. Recupera il nome del personaggio attivo
        string nomePersonaggio = "Utente"; // Fallback se qualcosa manca
        if (characterSelector != null)
        {
            var activeChar = characterSelector.GetActiveCharacter();
            if (activeChar != null)
            {
                nomePersonaggio = activeChar.nome.ToUpper() + " " + activeChar.cognome.ToUpper();
            }
        }

        // 2. Invia a ROS (Inviamo solo il contenuto o tutto? Di solito a ROS si manda il contenuto pulito)
        StringMsg msg = new StringMsg(messageContent);
        ros.Publish(topicName, msg);
        Debug.Log($"Inviato a ROS da {nomePersonaggio}: {messageContent}");

        // 3. Scrivi nella Chat di Unity
        if (chatDisplay != null)
        {
            string testoFormattato = $"<color=red><b>{nomePersonaggio}</b></color>: {messageContent}";
            chatDisplay.UpdateText(new StringMsg(testoFormattato));
        }

        // 4. Pulisci input
        if (inputFieldToClear != null)
        {
            inputFieldToClear.text = "";
            inputFieldToClear.ActivateInputField();
        }
    }
}