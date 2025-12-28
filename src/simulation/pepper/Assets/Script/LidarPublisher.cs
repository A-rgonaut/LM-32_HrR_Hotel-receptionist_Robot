using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Necessario per LaserScan
using System.Collections.Generic;

public class LidarPublisher : MonoBehaviour
{
    // Configurazioni ROS
    [Header("ROS Settings")]
    public string topicName = "scan";
    public float publishFrequency = 10f; // Hz (es. 10 volte al secondo)

    // Configurazioni LiDAR
    [Header("Lidar Settings")]
    public float maxDistance = 10f;       // Distanza massima del laser (metri)
    public float fieldOfView = 360f;      // Angolo di visione (es. 360 per giro completo)
    public int numRays = 360;             // Risoluzione: numero di raggi per scansione
    public float scanHeight = 0f;         // Offset verticale se necessario

    // Variabili interne
    private ROSConnection ros;
    private float timeElapsed;
    private float publishInterval;

    void Start()
    {
        // Ottieni l'istanza del connettore ROS
        ros = ROSConnection.GetOrCreateInstance();

        // Registra il topic
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        // Calcola l'intervallo di pubblicazione
        publishInterval = 1.0f / publishFrequency;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishInterval)
        {
            PerformScanAndPublish();
            timeElapsed = 0;
        }
    }

    void PerformScanAndPublish()
    {
        // 1. Preparazione dell'array per le distanze
        float[] ranges = new float[numRays];
        float angleIncrement = fieldOfView / numRays;
        float startAngle = -fieldOfView / 2.0f; // Inizia da sinistra

        // Messaggio di debug per la console (costruiamo una stringa)
        string consoleOutput = $"[Lidar] Scansione inviata a '{topicName}'. Rilevamenti vicini (< 2m): ";
        bool objectDetected = false;

        // 2. Ciclo di Raycasting
        for (int i = 0; i < numRays; i++)
        {
            float currentAngle = startAngle + (i * angleIncrement);

            // Calcolo la direzione del raggio rispetto alla rotazione attuale del sensore
            // Ruotiamo il vettore 'Forward' (asse Z) sull'asse Y
            Quaternion rotation = Quaternion.Euler(0, currentAngle, 0);
            Vector3 rayDirection = transform.rotation * rotation * Vector3.forward;

            // Punto di origine leggermente alzato se necessario
            Vector3 rayOrigin = transform.position + new Vector3(0, scanHeight, 0);

            // Esegui il Raycast
            if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, maxDistance))
            {
                ranges[i] = hit.distance; // Abbiamo colpito qualcosa

                // Disegna il raggio nella scena (solo per debug visivo in Unity)
                Debug.DrawRay(rayOrigin, rayDirection * hit.distance, Color.red);

                // LOGICA CONSOLE: Se l'oggetto è vicino, lo scriviamo
                if (hit.distance < 2.0f)
                {
                    objectDetected = true;
                    // Aggiungiamo info solo ogni tanto per non intasare, o solo il primo trovato
                    // Qui semplifico scrivendo solo se troviamo qualcosa
                }
            }
            else
            {
                ranges[i] = float.PositiveInfinity; // Nessun oggetto colpito (simula distanza infinita)
                Debug.DrawRay(rayOrigin, rayDirection * maxDistance, Color.green);
            }
        }

        // 3. Output su Console Unity
        if (objectDetected)
        {
            Debug.Log(consoleOutput + "RILEVATO OSTACOLO!");
        }
        else
        {
            // Commenta questa riga se è troppo "spam" nella console
            // Debug.Log("[Lidar] Nessun ostacolo vicino.");
        }

        // 4. Creazione e invio messaggio ROS
        LaserScanMsg scanMsg = new LaserScanMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "lidar_link", // Importante per TF
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                }
            },
            angle_min = startAngle * Mathf.Deg2Rad, // Convertire in radianti
            angle_max = (startAngle + fieldOfView) * Mathf.Deg2Rad,
            angle_increment = angleIncrement * Mathf.Deg2Rad,
            time_increment = 0f, // Istantaneo per simulazione
            scan_time = publishInterval,
            range_min = 0.1f,
            range_max = maxDistance,
            ranges = ranges
        };

        ros.Publish(topicName, scanMsg);
    }
}