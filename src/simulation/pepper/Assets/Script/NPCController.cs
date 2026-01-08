using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;

public class NPCController : MonoBehaviour
{
    [Header("Impostazioni Movimento")]
    public NavMeshAgent agent;
    public float waitTime = 2.0f;

    [Header("Punti Corridoio (Loop)")]
    public Transform[] corridorPoints;

    [Header("Punti Stanze (Opzionali)")]
    public Transform[] roomPoints;

    [Header("Probabilità")]
    [Range(0, 100)]
    public int roomVisitChance = 30;

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

        // NUOVO: Se non stiamo pattugliando (siamo in modalità manuale), 
        // usciamo dall'Update per non far ripartire la logica automatica.
        if (!isPatrolling) return;

        if (isWaiting) return;

        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            StartCoroutine(WaitAndDecide());
        }
    }

    // --- NUOVO METODO: Chiamato dal CharacterSelector ---
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
    }

    // (Opzionale) Se volessi farlo riprendere a pattugliare in futuro
    public void ResumePatrol()
    {
        isPatrolling = true;
        MoveToNextCorridorPoint();
    }
    // ----------------------------------------------------

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

            if (randomValue < roomVisitChance && roomPoints.Length > 0)
            {
                visitingRoom = true;
                MoveToRandomRoom();
            }
            else
            {
                currentCorridorIndex = (currentCorridorIndex + 1) % corridorPoints.Length;
                MoveToNextCorridorPoint();
            }
        }
        isWaiting = false;
    }

    void MoveToNextCorridorPoint()
    {
        if (corridorPoints.Length == 0) return;
        agent.SetDestination(corridorPoints[currentCorridorIndex].position);
    }

    void MoveToRandomRoom()
    {
        int randomIndex = Random.Range(0, roomPoints.Length);
        agent.SetDestination(roomPoints[randomIndex].position);
    }
}