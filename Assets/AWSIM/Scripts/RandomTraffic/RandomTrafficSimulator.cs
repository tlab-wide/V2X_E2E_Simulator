using UnityEngine;
using System;
using System.Collections.Generic;

namespace AWSIM.TrafficSimulation
{
    [Serializable]
    public struct RandomTrafficSimulatorConfiguration 
    {
        /// <summary>
        /// Available NPC prefabs
        /// </summary>
        [Tooltip("NPCs to be spawned.")]
        public GameObject[] npcPrefabs;

        [Serializable]
        public struct SpawnableLaneConfig
        {
            [Tooltip("Lane used for spawning.")]
            public TrafficLane lane;
            [Tooltip("Desired spawns per minute on this lane (0 = no limit).")]
            public float spawnsPerMinute;
        }

        [Tooltip("TrafficLanes where NPC vehicles can spawn, with optional per-lane rates.")]
        public SpawnableLaneConfig[] spawnableLanes;

        [Serializable]
        public struct BranchWeight
        {
            public TrafficLane nextLane;
            public float weight;
        }

        [Serializable]
        public struct BranchWeightSet
        {
            [Tooltip("Lane whose next branches are weighted.")]
            public TrafficLane fromLane;
            public BranchWeight[] next;
        }

        [Tooltip("Optional branch weights per lane; if empty, next lanes are chosen evenly.")]
        public BranchWeightSet[] branchWeights;

        [Tooltip("Describes the lifetime of a traffic simulator instance by specifying how many vehicles this traffic simulator will spawn. Setting it makes the spawner live longer or shorter, while it can also be set to infinity if needed (endless lifetime).")]
        public int maximumSpawns;

        [Tooltip("Is this traffic simulation enabled.")]
        public bool enabled;
    }

    /// <summary>
    /// Randomly simulate NPC traffic. (Currently only Vehicle is supported)
    /// - Automatic generation of NPC routes from VectorMap
    /// - Random NPC traffic
    /// - Traffic light control based on the Vienna Convention
    /// - Reproducibility by Seed value
    /// </summary>
    public class RandomTrafficSimulator : ITrafficSimulator
    {
        [SerializeField, Tooltip("Is the traffic enabled")]
        public bool enabled = true;

        private int maximumSpawns = 0;

        private NPCVehicleSimulator npcVehicleSimulator;

        private NPCVehicleSpawner npcVehicleSpawner;
        private int currentSpawnNumber = 0;

        private int spawnPriority = 0;

        private GameObject nextPrefabToSpawn = null;

        public void IncreasePriority(int priority)
        {
            spawnPriority += priority;
        }

        public int GetCurrentPriority()
        {
            return spawnPriority;
        }

        public void ResetPriority() {
            spawnPriority = 0;
        }

        public bool IsEnabled()
        {
            // TODO an interface to disable random traffic
            return enabled && !IsMaximumSpawnsNumberReached();
        }

        public RandomTrafficSimulator(GameObject parent,
            GameObject[] prefabs,
            RandomTrafficSimulatorConfiguration.SpawnableLaneConfig[] spawnableLanes,
            NPCVehicleSimulator vehicleSimulator,
            int maxSpawns = 0)
        {
            maximumSpawns = maxSpawns;
            npcVehicleSimulator = vehicleSimulator;
            npcVehicleSpawner = new NPCVehicleSpawner(parent, prefabs, spawnableLanes);
        }

        // Legacy overload for tests/older callers using TrafficLane[].
        public RandomTrafficSimulator(GameObject parent,
            GameObject[] prefabs,
            TrafficLane[] spawnableLanes,
            NPCVehicleSimulator vehicleSimulator,
            int maxSpawns = 0)
            : this(parent, prefabs, ConvertToConfigs(spawnableLanes), vehicleSimulator, maxSpawns)
        {
        }

        private static RandomTrafficSimulatorConfiguration.SpawnableLaneConfig[] ConvertToConfigs(TrafficLane[] lanes)
        {
            if (lanes == null) return Array.Empty<RandomTrafficSimulatorConfiguration.SpawnableLaneConfig>();
            var list = new List<RandomTrafficSimulatorConfiguration.SpawnableLaneConfig>();
            foreach (var lane in lanes)
            {
                if (lane == null) continue;
                list.Add(new RandomTrafficSimulatorConfiguration.SpawnableLaneConfig
                {
                    lane = lane,
                    spawnsPerMinute = 0f
                });
            }
            return list.ToArray();
        }

        public void GetRandomSpawnInfo(out NPCVehicleSpawnPoint spawnPoint, out GameObject prefab)
        {
            // NPC prefab is randomly chosen and is fixed until it is spawned. This is due to avoid prefab "bound" race conditions
            // when smaller cars will always be chosen over larger ones while the spawning process checks if the given prefab can be 
            // put in the given position. 
            if (nextPrefabToSpawn == null)
            {
                nextPrefabToSpawn = npcVehicleSpawner.GetRandomPrefab();
            }
            prefab = nextPrefabToSpawn;

            // Spawn position is always chosen randomly from the set of all available lanes.
            // This avoids waiting for lanes which are already occupied, but there is another one available somewhere else.
            spawnPoint = npcVehicleSpawner.GetRandomSpawnPoint();
        }

        public bool Spawn(GameObject prefab, NPCVehicleSpawnPoint spawnPoint, out NPCVehicle spawnedVehicle)
        {
            if(IsMaximumSpawnsNumberReached()) { 
                spawnedVehicle = null;
                return false;
            };

            if (npcVehicleSimulator.VehicleStates.Count >= npcVehicleSimulator.maxVehicleCount)
            {
                spawnedVehicle = null;
                return false;
            }

            var vehicle = npcVehicleSpawner.Spawn(prefab, SpawnIdGenerator.Generate(), spawnPoint);
            npcVehicleSimulator.Register(vehicle, spawnPoint.Lane, spawnPoint.WaypointIndex);
            nextPrefabToSpawn = null;

            if(maximumSpawns > 0)
                currentSpawnNumber++;

            spawnedVehicle = vehicle;
            return true;
        }

        private bool IsMaximumSpawnsNumberReached()
        {
            return (currentSpawnNumber == maximumSpawns && maximumSpawns > 0);
        }
    }
}
