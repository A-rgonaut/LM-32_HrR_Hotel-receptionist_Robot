using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Posizione Camera")]
    public Vector3 positionOffset;
    public float smoothSpeed = 5f;

    [Header("Gestione Rotazione")]
    public Vector3 lookAtOffset;
    public Vector3 rotationAdjustment;
    public float rotationSpeed = 5f;

    void LateUpdate()
    {
        if (target == null) return;

        // --- 1. POSIZIONE ---
        Vector3 desiredPosition = target.TransformPoint(positionOffset);
        transform.position = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed * Time.deltaTime);

        // --- 2. ROTAZIONE STABILIZZATA ---
        Vector3 targetPoint = target.position + lookAtOffset;
        Vector3 direction = targetPoint - transform.position;

        // Evitiamo calcoli se siamo troppo vicini (causa tremolii)
        if (direction.sqrMagnitude > 0.01f)
        {
            // Calcoliamo la rotazione grezza
            Quaternion lookRot = Quaternion.LookRotation(direction, Vector3.up);

            // TRUCCO: Convertiamo in angoli Euleriani (X, Y, Z) per pulire i dati
            Vector3 currentEuler = lookRot.eulerAngles;

            // Forza l'asse Z a 0 (niente avvitamenti!)
            currentEuler.z = 0;

            // Aggiungiamo il tuo aggiustamento manuale
            currentEuler += rotationAdjustment;

            // Riconvertiamo in rotazione finale
            Quaternion finalRotation = Quaternion.Euler(currentEuler);

            // Applichiamo morbidamente
            transform.rotation = Quaternion.Slerp(transform.rotation, finalRotation, rotationSpeed * Time.deltaTime);
        }
    }
}