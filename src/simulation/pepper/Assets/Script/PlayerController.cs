using UnityEngine;

public class PlayerController : MonoBehaviour
{
    [Header("Impostazioni Movimento")]
    public float speed = 5.0f;          // Velocità di movimento
    public float mouseSensitivity = 2.0f; // Sensibilità del mouse

    [Header("Limiti Telecamera")]
    public float maxLookAngle = 60.0f;  // 60 gradi su + 60 giù = 120 gradi totali

    private float rotationX = 0.0f;     // Variabile per tenere traccia della rotazione verticale
    private Camera playerCamera;        // Riferimento alla telecamera

    void Start()
    {
        // Trova la telecamera che è figlia di questo oggetto
        playerCamera = GetComponentInChildren<Camera>();

        // Blocca il cursore al centro dello schermo e lo nasconde
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    void Update()
    {
        // --- 1. Movimento (WASD) ---
        float moveX = Input.GetAxis("Horizontal"); // A e D
        float moveZ = Input.GetAxis("Vertical");   // W e S

        // Calcola il vettore di movimento relativo alla direzione in cui guarda il player
        Vector3 move = transform.right * moveX + transform.forward * moveZ;

        // Applica il movimento (usiamo Translate per un prototipo semplice senza fisica complessa)
        transform.Translate(move * speed * Time.deltaTime, Space.World);

        // --- 2. Rotazione Visuale (Mouse) ---
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;

        // A. Rotazione Orizzontale (Gira tutto il corpo/cilindro)
        transform.Rotate(Vector3.up * mouseX);

        // B. Rotazione Verticale (Gira solo la telecamera)
        rotationX -= mouseY;

        // Blocca la rotazione tra -60 e +60 gradi (Totale 120 gradi)
        rotationX = Mathf.Clamp(rotationX, -maxLookAngle, maxLookAngle);

        // Applica la rotazione alla sola telecamera
        if (playerCamera != null)
        {
            playerCamera.transform.localRotation = Quaternion.Euler(rotationX, 0.0f, 0.0f);
        }
    }
}