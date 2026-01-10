using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using TMPro; 
using RosMessageTypes.Std; 

public class RosSubscriberText : MonoBehaviour
{
    public string topicName = "/unity/dialogo_robot"; 
    public TextMeshProUGUI textDisplay;   

    void Start()
    {
        // Ci iscriviamo al topic.
        // Specifichiamo <StringMsg> come tipo di messaggio (std_msgs/String)
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(topicName, UpdateText);
    }

    public void UpdateText(StringMsg msg)
    {
        // Questa funzione viene chiamata ogni volta che arriva un messaggio:
        // Unity richiede che le modifiche alla UI avvengano sul thread principale
        textDisplay.text += msg.data + "\n";

        Debug.Log("Ricevuto da ROS 2: " + msg.data);

        //Canvas.ForceUpdateCanvases();
        //scrollRect.verticalNormalizedPosition = 0f;
    }
}
