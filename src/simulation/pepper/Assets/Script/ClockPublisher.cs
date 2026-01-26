using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Rosgraph;

public class ClockPublisher : MonoBehaviour {
    ROSConnection ros;
    public string topicName = "/clock";
    public float publishRate = 100f;

    private float timeElapsed;

    void Start() {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ClockMsg>(topicName);
    }

    void Update() {
        timeElapsed += Time.deltaTime;
        float interval = 1.0f / publishRate;
        if (timeElapsed >= interval) {
            PublishClock();
            timeElapsed -= interval;
        }
    }

    void PublishClock() {
        double time = Time.time;
        int sec = (int)time;
        uint nanosec = (uint)((time - sec) * 1e9);
        ClockMsg clockMsg = new ClockMsg {
            clock = new RosMessageTypes.BuiltinInterfaces.TimeMsg { sec = sec, nanosec = nanosec }
        };
        ros.Publish(topicName, clockMsg);
    }
}
