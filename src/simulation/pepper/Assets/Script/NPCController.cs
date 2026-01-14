using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;
// 1. Aggiungiamo i namespace ROS
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

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

    [Header("ROS Settings")]
    public string rosStateTopic = "/unity/stato"; // Topic da ascoltare

    private int currentCorridorIndex = 0;
    private bool isWaiting = false;
    private bool visitingRoom = false;
    private bool isPatrolling = true;

    private Animator animator;

    // Classe per leggere il JSON di ROS
    [System.Serializable]
    public class RosCommandData
    {
        public string comando;
        public string scenario;
    }

    void Start()
    {
        if (agent == null) agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        // Iscrizione al topic ROS
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(rosStateTopic, OnRosStateReceived);

        // Inizia il pattugliamento normale
        MoveToNextCorridorPoint();
    }

    void Update()
    {
        if (animator != null)
        {
            animator.SetFloat("Speed", agent.velocity.magnitude);
        }

        if (!isPatrolling) return;
        if (isWaiting) return;

        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            StartCoroutine(WaitAndDecide());
        }
    }

    //Funzione chiamata ogni volta che ROS invia un messaggio su /unity/stato
    void OnRosStateReceived(StringMsg msg)
    {
        // Se stiamo già pattugliando, ignoriamo i comandi di fine scenario
        if (isPatrolling) return;

        try
        {
            // Decodifica il JSON
            RosCommandData data = JsonUtility.FromJson<RosCommandData>(msg.data);

            // Controlla il comando
            if (data.comando == "FINE_SCENARIO")
            {
                Debug.Log($"[NPC] Ricevuto FINE_SCENARIO (Scenario: {data.scenario}). Torno a pattugliare.");

                // Opzionale: Puoi aggiungere un controllo su 'data.scenario' 
                // se vuoi che risponda solo a scenari specifici (es: if scenario == "A")

                ResumePatrol();
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Errore parsing JSON NPC: {e.Message}");
        }
    }

    public void GoToTargetAndStay(Transform targetPoint)
    {
        StopAllCoroutines();
        isPatrolling = false; // Entra in modalità Manuale/Scenario
        isWaiting = false;

        if (targetPoint != null)
        {
            agent.SetDestination(targetPoint.position);
        }

        Debug.Log("[NPC] Vado al target e attendo il comando ROS 'FINE_SCENARIO'...");
    }

    public void ResumePatrol()
    {
        // Riattiviamo il NavMesh se era stato fermato (es. da PerformDying)
        if (agent != null)
        {
            agent.isStopped = false;
        }

        isPatrolling = true;
        Debug.Log("[NPC] Riprendo il pattugliamento.");
        MoveToNextCorridorPoint();
    }

    public void PerformDying()
    {
        StopAllCoroutines();
        isPatrolling = false;
        isWaiting = false;

        if (agent != null && agent.isOnNavMesh)
        {
            agent.isStopped = true;
            agent.ResetPath();
            agent.velocity = Vector3.zero;
        }

        if (animator != null)
        {
            animator.SetTrigger("TriggerDying");
        }
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