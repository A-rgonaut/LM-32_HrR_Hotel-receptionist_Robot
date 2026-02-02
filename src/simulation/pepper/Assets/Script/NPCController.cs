using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.AI;

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
    public string rosStateTopic = "/unity/stato";

    private int currentCorridorIndex = 0;
    private bool isWaiting = false;
    private bool visitingRoom = false;
    private bool isPatrolling = true;
    private bool _rotateOnArrival = false;

    public bool isCriticalCondition = false;
    private bool isDancing = false;

    private enum StopState { None, Kneeling, Dying }
    private StopState currentStopState = StopState.None;

    // --- NUOVO: Variabile per tracciare il nome dello scenario corrente ---
    private string currentActiveScenario = "";
    // ---------------------------------------------------------------------

    private Animator animator;

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

        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(rosStateTopic, OnRosStateReceived);

        MoveToNextCorridorPoint();
    }

    void Update()
    {
        if (animator != null) animator.SetFloat("Speed", agent.velocity.magnitude);

        if (!isPatrolling && _rotateOnArrival)
        {
            if (!agent.pathPending && agent.remainingDistance <= agent.stoppingDistance)
            {
                if (!agent.hasPath || agent.velocity.sqrMagnitude == 0f)
                {
                    StartCoroutine(Rotate180());
                    _rotateOnArrival = false;
                }
            }
        }

        if (!isPatrolling) return;
        if (isWaiting) return;

        if (!agent.pathPending && agent.remainingDistance < 0.5f)
            StartCoroutine(WaitAndDecide());
    }

    // Callback ROS modificata per controllare lo scenario
    void OnRosStateReceived(StringMsg msg)
    {
        // Se stiamo già pattugliando, non c'è nulla da fermare
        if (isPatrolling) return;

        try
        {
            RosCommandData data = JsonUtility.FromJson<RosCommandData>(msg.data);

            if (data.comando == "FINE_SCENARIO")
            {
                if (data.scenario[^1] == currentActiveScenario[0])
                    ResumePatrol();
                else
                    Debug.LogWarning($"Ho interrotto 'Scenario {data.scenario[^1]}', per eseguire lo 'Scenario {currentActiveScenario}'");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Errore parsing JSON NPC: {e.Message}");
        }
    }

    // MODIFICATO: Ora accetta il nome dello scenario
    public void GoToTargetAndStay(Transform targetPoint, string scenarioID)
    {
        StopAllCoroutines();
        isPatrolling = false;
        isWaiting = false;

        // Impostiamo l'ID dello scenario (es. "A" o "B")
        currentActiveScenario = scenarioID;

        if (targetPoint != null)
        {
            agent.SetDestination(targetPoint.position);
        }

        Debug.Log($"[NPC] Avviato Scenario {scenarioID}. Vado al target...");
    }

    public void EnableRotationOnArrival()
    {
        _rotateOnArrival = true;
    }

    IEnumerator Rotate180()
    {
        Quaternion startRotation = transform.rotation;
        Quaternion endRotation = transform.rotation * Quaternion.Euler(0, 180, 0);

        float duration = 1.5f;
        float elapsed = 0.0f;

        while (elapsed < duration)
        {
            transform.rotation = Quaternion.Slerp(startRotation, endRotation, elapsed / duration);
            elapsed += Time.deltaTime;
            yield return null;
        }
        transform.rotation = endRotation;
    }

    public void PerformKneeling()
    {
        PrepareForStop();
        currentStopState = StopState.Kneeling;
        isCriticalCondition = false;

        // Impostiamo hardcoded "C" perché questa funzione è specifica per lo scenario C
        currentActiveScenario = "C";

        if (animator != null) animator.SetTrigger("TriggerKneel");
    }

    public void PerformDying()
    {
        PrepareForStop();
        currentStopState = StopState.Dying;
        isCriticalCondition = true;

        // Impostiamo hardcoded "C2" 
        currentActiveScenario = "C2";

        if (animator != null) animator.SetTrigger("TriggerDie");

        CapsuleCollider col = GetComponent<CapsuleCollider>();
        if (col != null) { col.height = 0.2f; col.center = new Vector3(0, 0.1f, 0); }
    }

    private void PrepareForStop()
    {
        StopAllCoroutines();
        isPatrolling = false;

        if (agent != null && agent.isOnNavMesh)
        {
            agent.isStopped = true;
            agent.ResetPath();
            agent.velocity = Vector3.zero;
        }
    }

    public void ResumePatrol()
    {
        isCriticalCondition = false;
        isPatrolling = true;
        currentActiveScenario = ""; // Reset dello scenario

        if (animator != null)
        {
            if (currentStopState == StopState.Kneeling)
            {
                animator.SetTrigger("RecoverKneel");
            }
            else if (currentStopState == StopState.Dying)
            {
                animator.SetTrigger("RecoverDie");
                CapsuleCollider col = GetComponent<CapsuleCollider>();
                if (col != null) { col.height = 1.8f; col.center = new Vector3(0, 0.9f, 0); }
            }
        }

        currentStopState = StopState.None;

        if (agent != null) agent.isStopped = false;

        MoveToNextCorridorPoint();
    }

    // ... Resto dei metodi (WaitAndDecide, MoveToRandomRoom, ToggleDance) uguali a prima ...
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

    public void ToggleDance()
    {
        isDancing = !isDancing;

        if (isDancing)
        {
            PrepareForStop();

            currentStopState = StopState.None;
            isCriticalCondition = false;
            currentActiveScenario = "EASTER_EGG"; // Giusto per completezza

            if (animator != null) animator.SetBool("IsDancing", true);
        }
        else
        {
            if (animator != null) animator.SetBool("IsDancing", false);

            ResumePatrol();
        }
    }
}