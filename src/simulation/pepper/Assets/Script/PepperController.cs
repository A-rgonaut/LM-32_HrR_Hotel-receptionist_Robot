using UnityEngine;
using UnityEngine.InputSystem; // Necessario per il nuovo sistema

public class PepperController : MonoBehaviour
{
    [Header("Impostazioni Movimento")]
    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;

    [Header("Riferimenti Ruote")]
    public Transform[] ruote;

    private Rigidbody rb;
    private Vector3 movementInput;
    private float rotationInput;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        // Controlliamo se esiste una tastiera collegata
        if (Keyboard.current == null) return;

        // Reset input
        float moveX = 0f;
        float moveZ = 0f;
        rotationInput = 0f;

        // --- Sintassi per il NUOVO Input System ---

        // W / S -> Avanti/Indietro
        if (Keyboard.current.wKey.isPressed) moveZ = 1f;
        if (Keyboard.current.sKey.isPressed) moveZ = -1f;

        // A / D -> Sinistra/Destra
        if (Keyboard.current.aKey.isPressed) moveX = -1f;
        if (Keyboard.current.dKey.isPressed) moveX = 1f;

        // Frecce -> Rotazione
        if (Keyboard.current.leftArrowKey.isPressed) rotationInput = -1f;
        if (Keyboard.current.rightArrowKey.isPressed) rotationInput = 1f;

        // Calcolo vettore movimento (identico a prima)
        movementInput = (transform.right * moveX + transform.forward * moveZ).normalized;

        RuotaMeshRuote(moveX, moveZ);
    }

    void FixedUpdate()
    {
        // Fisica identica a prima
        if (movementInput.magnitude > 0.1f)
        {
            Vector3 targetPosition = rb.position + movementInput * moveSpeed * Time.fixedDeltaTime;
            rb.MovePosition(targetPosition);
        }

        if (Mathf.Abs(rotationInput) > 0.1f)
        {
            float turn = rotationInput * rotationSpeed * Time.fixedDeltaTime;
            Quaternion turnRotation = Quaternion.Euler(0f, turn, 0f);
            rb.MoveRotation(rb.rotation * turnRotation);
        }
    }

    void RuotaMeshRuote(float x, float z)
    {
        if (ruote != null && ruote.Length > 0)
        {
            bool isMoving = x != 0 || z != 0;
            if (isMoving)
            {
                foreach (Transform ruota in ruote)
                {
                    if (ruota != null)
                        ruota.Rotate(Vector3.right * moveSpeed * 5 * Time.deltaTime);
                }
            }
        }
    }
}