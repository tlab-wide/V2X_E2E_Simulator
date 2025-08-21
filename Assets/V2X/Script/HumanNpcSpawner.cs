using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class HumanNpcSpawner : MonoBehaviour
{
    [Header("Pool & Waypoints")]
    [Tooltip("Pool of human NPC instances. All will be deactivated on start.")]
    public List<Transform> HumanPool = new List<Transform>();

    [Tooltip("List of waypoint systems to choose from (unique selection each cycle).")]
    public List<WaypointSystem> waypointSystems = new List<WaypointSystem>();

    [Header("Spawning Config")]
    [Tooltip("Time (seconds) between each random generation cycle.")]
    public float waitTimeRandomGeneration = 5f;

    [Tooltip("How many humans to spawn each cycle (exact, unless variance is used).")]
    public int numberSpawningHuman = 3;

    [Header("Randomization")]
    [Tooltip("Random additional +/- humans around 'numberSpawningHuman' each cycle.")]
    public int varianceSpawningHuman = 0; // set >0 if you want jitter

    [Tooltip("Max additional clones we are allowed to create if pool is short.")]
    public int maxExtraInstantiatesPerCycle = 10;

    [Tooltip("Optional parent for activated NPCs.")]
    public Transform activeParent;

    private Coroutine _spawnLoop;

    private void Start()
    {
        DeactivateEntirePool();
        _spawnLoop = StartCoroutine(SpawnLoop());
    }

    private void DeactivateEntirePool()
    {
        for (int i = 0; i < HumanPool.Count; i++)
        {
            if (HumanPool[i] != null)
                HumanPool[i].gameObject.SetActive(false);
        }
    }

    private IEnumerator SpawnLoop()
    {
        var wait = new WaitForSeconds(Mathf.Max(0.01f, waitTimeRandomGeneration));

        while (true)
        {
            yield return wait;

            int targetCount = numberSpawningHuman;
            if (varianceSpawningHuman > 0)
            {
                int delta = Random.Range(-varianceSpawningHuman, varianceSpawningHuman + 1);
                targetCount = Mathf.Max(0, numberSpawningHuman + delta);
            }

            if (targetCount == 0) continue;

            Debug.Log($"number of spawning humans 1: {numberSpawningHuman}");
            SpawnBatch(targetCount);
            
        }
    }

    private void SpawnBatch(int count)
    {
        // We can’t assign more pedestrians than unique waypoint systems available.
        int maxPossible = Mathf.Min(count, waypointSystems.Count);
        Debug.Log($"number of spawning humans 2: {maxPossible}");
        if (maxPossible == 0) return;

        // Unique waypoint systems for this batch (no replacement).
        List<WaypointSystem> selectedWaypoints = TakeRandomDistinct(waypointSystems, maxPossible);
        Debug.Log($"number of spawning humans 3: {selectedWaypoints.Count}");

        // Pull deactivated NPCs from pool (random, without replacement).
        List<Transform> deactivated = HumanPool.Where(t => t != null && !t.gameObject.activeSelf).ToList();
        int canTake = Mathf.Min(maxPossible, deactivated.Count);
        List<Transform> chosenFromPool = TakeRandomDistinct(deactivated, canTake);

        // If short, instantiate extra NPCs by cloning random templates from the pool.
        int shortage = maxPossible - chosenFromPool.Count;
        List<Transform> instantiated = new List<Transform>();
        if (shortage > 0)
        {
            int allowed = Mathf.Min(shortage, maxExtraInstantiatesPerCycle);
            instantiated = InstantiateRandomFromPool(allowed);
            HumanPool.AddRange(instantiated);
        }

        // Final list to activate; this list size equals selectedWaypoints size.
        List<Transform> toActivate = new List<Transform>(chosenFromPool);
        toActivate.AddRange(instantiated);
        // toActivate.ForEach(t  => Debug.Log($"name of the selected {t.gameObject.name}") );

        for (int i = 0; i < toActivate.Count; i++)
        {
            var npc = toActivate[i];
            var wpSystem = selectedWaypoints[i];
            if (npc == null || wpSystem == null) continue;

            InitializeFollowerAtStart(npc, wpSystem);

            if (activeParent != null)
                npc.SetParent(activeParent, true);

            npc.gameObject.SetActive(true);
        }
    }

    /// <summary>
    /// Assigns the selected WaypointSystem to the NPC's WaypointFollower and forces start at index 0.
    /// Assumes every NPC already has a WaypointFollower component (as per your note).
    /// </summary>
    private void InitializeFollowerAtStart(Transform npc, WaypointSystem wpSystem)
    {
        var follower = npc.GetComponent<WaypointFollower>();
        if (follower == null)
        {
            Debug.LogWarning($"WaypointFollower missing on {npc.name}, but was expected.");
            return;
        }

        // Assign the chosen system.
        follower.SetWaypointSystem(wpSystem);

        // If you added the helper on WaypointFollower, use it:
        // (Recommended for explicitly setting index = 0 and snapping to first waypoint.)
        follower.ForceStartAtFirstWaypoint(); // requires the helper method you added

        // If you *haven’t* added ForceStartAtFirstWaypoint yet, comment the line above and use this fallback:
        // Transform firstWp = wpSystem.GetFirstWaypoint();
        // npc.SetPositionAndRotation(firstWp.position, firstWp.rotation);
        // var rb = npc.GetComponent<Rigidbody>();
        // if (rb != null) { rb.linearVelocity = Vector3.zero; rb.angularVelocity = Vector3.zero; }
        // // TODO: Without the helper, currentWaypointIndex is private; rely on WaypointFollower.Start() closest logic.
    }

    // --- Helpers ---

    private List<T> TakeRandomDistinct<T>(List<T> list, int takeCount)
    {
        if (takeCount <= 0) return new List<T>();
        if (takeCount >= list.Count) return new List<T>(Shuffle(list));
        List<T> shuffled = Shuffle(list);
        return shuffled.GetRange(0, takeCount);
    }

    private List<Transform> InstantiateRandomFromPool(int count)
    {
        List<Transform> created = new List<Transform>();
        if (count <= 0) return created;

        var validTemplates = HumanPool.Where(t => t != null).ToList();
        if (validTemplates.Count == 0) return created;

        for (int i = 0; i < count; i++)
        {
            Transform template = validTemplates[Random.Range(0, validTemplates.Count)];
            if (template == null) continue;

            GameObject cloneGO = Instantiate(template.gameObject);
            cloneGO.name = template.gameObject.name + "_Clone";
            cloneGO.SetActive(false); // keep deactivated until positioned

            Transform cloneT = cloneGO.transform;
            cloneT.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            cloneT.localScale = template.localScale;

            created.Add(cloneT);
        }
        return created;
    }

    private List<T> Shuffle<T>(List<T> source)
    {
        for (int i = 0; i < source.Count; i++)
        {
            int j = Random.Range(i, source.Count);
            (source[i], source[j]) = (source[j], source[i]);
        }
        return source;
    }

    // --- Public controls ---

    /// <summary>Immediately triggers a spawn cycle with the current settings.</summary>
    public void TriggerNow()
    {
        int targetCount = numberSpawningHuman;
        if (varianceSpawningHuman > 0)
        {
            int delta = Random.Range(-varianceSpawningHuman, varianceSpawningHuman + 1);
            targetCount = Mathf.Max(0, numberSpawningHuman + delta);
        }
        SpawnBatch(targetCount);
    }

    /// <summary>Stops and restarts the spawn loop (useful if you change waitTimeRandomGeneration at runtime).</summary>
    public void RestartLoop()
    {
        if (_spawnLoop != null) StopCoroutine(_spawnLoop);
        _spawnLoop = StartCoroutine(SpawnLoop());
    }
}
