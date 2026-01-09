using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;

public class NPCController : MonoBehaviour
{
    [Header("Movement settings")]
    public NavMeshAgent agent;
    public float waitTime = 2.0f;

    [Header("Waypoints (Loop)")]
    public Transform[] waypointsLoop;

    [Header("Waypoints (Optional)")]
    public Transform[] waypointsOptional;

    [Header("Probability")]
    [Range(0, 100)]
    public int waypointsOptionalChance = 30;

    private int currentCorridorIndex = 0;
    private bool isWaiting = false;
    private bool visitingRoom = false;

    // NUOVO: Variabile per sapere se siamo in modalità manuale
    private bool isPatrolling = true;

    private Animator animator;

    void Start()
    {
        if (agent == null) agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        // Inizia il pattugliamento normale
        MoveToNextCorridorPoint();
    }

    void Update()
    {
        if (animator != null)
        {
            animator.SetFloat("Speed", agent.velocity.magnitude);
        }

        // Se non stiamo pattugliando (siamo in modalità manuale), 
        // usciamo dall'Update per non far ripartire la logica automatica.
        if (!isPatrolling) return;

        if (isWaiting) return;

        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            StartCoroutine(WaitAndDecide());
        }
    }

    public void GoToTargetAndStay(Transform targetPoint)
    {
        // 1. Fermiamo qualsiasi ragionamento in corso (coroutine di attesa)
        StopAllCoroutines();

        // 2. Disattiviamo la logica di pattugliamento automatica
        isPatrolling = false;
        isWaiting = false;

        // 3. Impostiamo la destinazione specifica
        if (targetPoint != null)
        {
            agent.SetDestination(targetPoint.position);
        }

        // 4. TODO: implementare logica di "soddisfacimento" e
        // ritornare al pattugliamento
    }

    // (Opzionale) Se si volesse farlo riprendere a pattugliare in futuro
    public void ResumePatrol()
    {
        isPatrolling = true;
        MoveToNextCorridorPoint();
    }

    public void PerformDying()
    {
        // 1. Ferma logiche di pattugliamento o attesa
        StopAllCoroutines();
        isPatrolling = false;
        isWaiting = false;

        // 2. Blocca fisicamente il movimento del NavMesh
        if (agent != null && agent.isOnNavMesh)
        {
            agent.isStopped = true; // Ferma l'agente
            agent.ResetPath();      // Cancella il percorso corrente
            agent.velocity = Vector3.zero; // Azzera la velocità residua
        }

        // 3. Attiva l'animazione di caduta
        if (animator != null)
        {
            animator.SetTrigger("TriggerDying");
        }

        // Opzionale: Disabilitare il collider se serve che altri ci passino sopra,
        // ma per ora lasciamolo così per semplicità.
    }

    System.Collections.IEnumerator WaitAndDecide()
    {
        isWaiting = true;
        yield return new WaitForSeconds(waitTime);

        if (visitingRoom)
        {
            visitingRoom = false;
            MoveToNextCorridorPoint();
        }
        else
        {
            int randomValue = Random.Range(0, 100);

            if (randomValue < waypointsOptionalChance && waypointsOptional.Length > 0)
            {
                visitingRoom = true;
                MoveToRandomRoom();
            }
            else
            {
                currentCorridorIndex = (currentCorridorIndex + 1) % waypointsLoop.Length;
                MoveToNextCorridorPoint();
            }
        }
        isWaiting = false;
    }

    void MoveToNextCorridorPoint()
    {
        if (waypointsLoop.Length == 0) return;
        agent.SetDestination(waypointsLoop[currentCorridorIndex].position);
    }

    void MoveToRandomRoom()
    {
        int randomIndex = Random.Range(0, waypointsOptional.Length);
        agent.SetDestination(waypointsOptional[randomIndex].position);
    }
}