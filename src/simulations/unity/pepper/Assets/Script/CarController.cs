using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("Impostazioni Guida")]
    public float motorTorque = 1500f; // Potenza del motore
    public float maxSteerAngle = 30f; // Angolo massimo di sterzata
    public float brakeForce = 3000f;  // Forza frenante (opzionale per stabilità)

    [Header("Wheel Colliders (Fisica)")]
    public WheelCollider frontLeftWC;
    public WheelCollider frontRightWC;
    public WheelCollider rearLeftWC;
    public WheelCollider rearRightWC;

    [Header("Wheel Meshes (Grafica)")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    void FixedUpdate()
    {
        GetInput();
    }

    void GetInput()
    {
        // Lettura Input W/S (Vertical) e A/D (Horizontal)
        float v = Input.GetAxis("Vertical");
        float h = Input.GetAxis("Horizontal");

        // 1. MOTORE (Solo sulle ruote anteriori come richiesto)
        frontLeftWC.motorTorque = v * motorTorque;
        frontRightWC.motorTorque = v * motorTorque;

        // 2. STERZO (Solo sulle ruote anteriori)
        float steer = h * maxSteerAngle;
        frontLeftWC.steerAngle = steer;
        frontRightWC.steerAngle = steer;

        // 3. AGGIORNAMENTO GRAFICO (Visualizzazione rotazione ruote)
        UpdateWheelVisuals(frontLeftWC, frontLeftMesh);
        UpdateWheelVisuals(frontRightWC, frontRightMesh);
        UpdateWheelVisuals(rearLeftWC, rearLeftMesh);
        UpdateWheelVisuals(rearRightWC, rearRightMesh);
    }

    // Funzione magica che fa girare le mesh 3D come i collider fisici
    void UpdateWheelVisuals(WheelCollider collider, Transform wheelMesh)
    {
        Vector3 position;
        Quaternion rotation;

        // Ottiene la posizione e rotazione calcolata dalla fisica
        collider.GetWorldPose(out position, out rotation);

        // Applica alla ruota visibile
        wheelMesh.position = position;
        wheelMesh.rotation = rotation;
    }
}