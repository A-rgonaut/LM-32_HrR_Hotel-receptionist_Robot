using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Per il messaggio String

public class RosTextPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "chat_topic";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    // Questa è la funzione che apparirà nel menu "On End Edit"
    public void PublishMessagfe(string messageContent)
    {
        StringMsg msg = new StringMsg(messageContent);
        ros.Publish(topicName, msg);
        Debug.Log("Inviato a ROS: " + messageContent);
    }
}