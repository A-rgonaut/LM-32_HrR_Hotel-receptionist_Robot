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
    public float scanHeight = 0.30f;         // Offset verticale se necessario

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


        // ======================= AGGIUNGI QUESTO LOG =======================
        // Calcola l'altezza esatta rispetto al mondo (assumendo pavimento a Y=0)
        float altezzaMondo = transform.position.y;

        // Cerca di capire se c'è scaling strano nei parent
        float scalaRealeY = transform.lossyScale.y;

        Debug.LogWarning($"[DEBUG LIDAR] ==============================================");
        Debug.LogWarning($"[DEBUG LIDAR] Altezza REALE dal suolo (World Y): {altezzaMondo} metri");
        Debug.LogWarning($"[DEBUG LIDAR] Posizione Locale (Local Y): {transform.localPosition.y}");
        Debug.LogWarning($"[DEBUG LIDAR] Scala Globale (LossyScale Y): {scalaRealeY} (Se non è 1, occhio!)");
        Debug.LogWarning($"[DEBUG LIDAR] ==============================================");
        // ===================================================================
    }
    void OnDestroy()
    {
        ros.Disconnect();
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
        // string consoleOutput = $"[Lidar] Scansione inviata a '{topicName}'. Rilevamenti vicini (< 2m): ";
        // bool objectDetected = false;
        Dictionary<string, float> oggettiVisti = new Dictionary<string, float>();

        // 2. Ciclo di Raycasting
        for (int i = 0; i < numRays; i++)
        {
            float currentAngle = startAngle + (i * angleIncrement);

            // Calcolo la direzione del raggio rispetto alla rotazione attuale del sensore
            // Ruotiamo il vettore 'Forward' (asse Z) sull'asse Y
            Quaternion rotation = Quaternion.Euler(0, -currentAngle, 0);
            Vector3 rayDirection = transform.rotation * rotation * Vector3.forward;

            // Punto di origine leggermente alzato se necessario
            Vector3 rayOrigin = transform.position + new Vector3(0, scanHeight, 0);

            // Esegui il Raycast
            if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, maxDistance))
            {
                ranges[i] = hit.distance; // Abbiamo colpito qualcosa
                string nomeOggetto = hit.collider.gameObject.name;
                if (!oggettiVisti.ContainsKey(nomeOggetto)) {
                    oggettiVisti.Add(nomeOggetto, hit.distance);
                } else if (hit.distance < oggettiVisti[nomeOggetto]) {
                    oggettiVisti[nomeOggetto] = hit.distance;
                }

                // Disegna il raggio nella scena (solo per debug visivo in Unity)
                Debug.DrawRay(rayOrigin, rayDirection * hit.distance, Color.red);

                // LOGICA CONSOLE: Se l'oggetto è vicino, lo scriviamo
                /* if (hit.distance < 2.0f)
                {
                    objectDetected = true;
                    // Aggiungiamo info solo ogni tanto per non intasare, o solo il primo trovato
                    // Qui semplifico scrivendo solo se troviamo qualcosa
                } */
            }
            else
            {
                ranges[i] = maxDistance;  // Nessun oggetto colpito
                Debug.DrawRay(rayOrigin, rayDirection * maxDistance, Color.green);
            }
        }

        // 3. Output su Console Unity
        if (oggettiVisti.Count > 0)  // if (objectDetected)
        {
            // Debug.Log(consoleOutput + "RILEVATO OSTACOLO!");
            Debug.Log($"[LIDAR VEDE]: {string.Join(", ", oggettiVisti)}");
        }
        else
        {
            // Commenta questa riga se è troppo "spam" nella console
            // Debug.Log("[Lidar] Nessun ostacolo vicino.");
        }

        // 4. Creazione e invio messaggio ROS
        double time = Time.time;
        int sec = (int)time;
        uint nanosec = (uint)((time - sec) * 1e9);
        LaserScanMsg scanMsg = new LaserScanMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "lidar_link", // Importante per TF
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = sec,
                    nanosec = nanosec
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
