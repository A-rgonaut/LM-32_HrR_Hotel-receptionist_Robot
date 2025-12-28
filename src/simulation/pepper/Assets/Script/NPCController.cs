using UnityEngine;
using UnityEngine.AI; // FONDAMENTALE per il NavMesh
using System.Collections.Generic;

public class NPCController : MonoBehaviour
{
    [Header("Impostazioni Movimento")]
    public NavMeshAgent agent;
    public float waitTime = 2.0f; // Tempo di attesa quando arriva a destinazione

    [Header("Punti Corridoio (Loop)")]
    public Transform[] corridorPoints;

    [Header("Punti Stanze (Opzionali)")]
    public Transform[] roomPoints;

    [Header("Probabilità")]
    [Range(0, 100)]
    public int roomVisitChance = 30; // 30% di probabilità di entrare in una stanza

    private int currentCorridorIndex = 0;
    private bool isWaiting = false;
    private bool visitingRoom = false;

    private Animator animator;

    void Start()
    {
        if (agent == null) agent = GetComponent<NavMeshAgent>();

        animator = GetComponent<Animator>();

        // Inizia andando al primo punto del corridoio
        MoveToNextCorridorPoint();
    }

    void Update()
    {
        if (animator != null)
        {
            animator.SetFloat("Speed", agent.velocity.magnitude);
        }
        // Se sta aspettando, non fare nulla
        if (isWaiting) return;

        // Controlla se l'agente è arrivato a destinazione
        // (remainingDistance diventa molto basso quando è arrivato)
        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            StartCoroutine(WaitAndDecide());
        }
    }

    System.Collections.IEnumerator WaitAndDecide()
    {
        isWaiting = true;

        // Aspetta un po' (simula che si guardi in giro)
        yield return new WaitForSeconds(waitTime);

        // DECISIONE: Dove vado adesso?

        if (visitingRoom)
        {
            // Se ero in una stanza, devo tornare al corridoio
            visitingRoom = false;
            MoveToNextCorridorPoint();
        }
        else
        {
            // Se sono nel corridoio, tiro un dado
            int randomValue = Random.Range(0, 100);

            if (randomValue < roomVisitChance && roomPoints.Length > 0)
            {
                // VINTO: Entra in una stanza a caso
                visitingRoom = true;
                MoveToRandomRoom();
            }
            else
            {
                // PERSO: Continua il giro del corridoio
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