using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// This class simulates states of NPC vehicles and updates visualization via <see cref="NPCVehicle"/>.
    /// The process of the simulation consists of three steps:<br/>
    /// - Cognition step implemented in <see cref="NPCVehicleCognitionStep"/><br/>
    /// - Decision step implemented in <see cref="NPCVehicleDecisionStep"/><br/>
    /// - Control step implemented in <see cref="NPCVehicleControlStep"/><br/>
    /// Each step updates <see cref="NPCVehicleInternalState"/> and the result is passed to <see cref="NPCVehicleVisualizationStep"/> for visualization update.
    /// </summary>
    /// <summary>
    /// Rules that keep the traffic moving.
    /// </summary>
    [Serializable]
    public struct NPCTrafficBehaviour
    {
        [Tooltip("A vehicle that stands still this long while yielding gets the right of way (s). 0 = off.")]
        public float deadlockTimeout;
        [Tooltip("How long a vehicle keeps the right of way it was given (s).")]
        public float forcedPassDuration;
        [Tooltip("Only one vehicle gets the right of way within this distance (m).")]
        public float forcedPassRadius;
        [Tooltip("A vehicle that stands still this long for any reason is removed (s). 0 = off.")]
        public float stuckDespawnTimeout;
        [Tooltip("Below this speed a vehicle counts as standing still (m/s).")]
        public float standstillSpeed;

        public static NPCTrafficBehaviour Default() => new NPCTrafficBehaviour
        {
            deadlockTimeout = 8f, forcedPassDuration = 6f, forcedPassRadius = 70f,
            stuckDespawnTimeout = 180f, standstillSpeed = 0.3f,
        };
    }

    public class NPCVehicleSimulator : IDisposable
    {
        public NPCTrafficBehaviour Behaviour { get; set; } = NPCTrafficBehaviour.Default();

        /// <summary>Number of vehicles removed because they were stuck, and right-of-way grants, since the start.</summary>
        public int StuckDespawnCount { get; private set; }
        public int ForcedPassCount { get; private set; }

        private float simulationTime;
        /// <summary>
        /// Get NPC vehicle states that are updated in simulation steps.<br/>
        /// </summary>
        public IReadOnlyList<NPCVehicleInternalState> VehicleStates
            => vehicleStates;

        /// <summary>
        /// Get or set EGO Vehicle that should be considered in the simulation.
        /// </summary>
        public Transform EGOVehicle { get; set; }

        public int maxVehicleCount;

        private List<NPCVehicleInternalState> vehicleStates;
        private NPCVehicleCognitionStep cognitionStep;
        private NPCVehicleDecisionStep decisionStep;
        private NPCVehicleControlStep controlStep;
        private NPCVehicleVisualizationStep visualizationStep;
        private Transform dummyEgo;

        public NPCVehicleSimulator(NPCVehicleConfig config,
            LayerMask vehicleLayerMask,
            LayerMask groundLayerMask,
            int maxVehicleCount,
            GameObject egoVehicle)
        {
            vehicleStates = new List<NPCVehicleInternalState>();
            cognitionStep = new NPCVehicleCognitionStep(vehicleLayerMask, groundLayerMask, maxVehicleCount);
            decisionStep = new NPCVehicleDecisionStep(config);
            controlStep = new NPCVehicleControlStep(config);
            visualizationStep = new NPCVehicleVisualizationStep();
            this.maxVehicleCount = maxVehicleCount;
            EGOVehicle = egoVehicle.transform;
        }


        /// <summary>
        /// When there is no real Ego vehicle in a scene, dummy one must be set.
        /// </summary>
        public void UnregisterEgo()
        {
            if (dummyEgo)
            {
                EGOVehicle = dummyEgo;
            }
        }

        /// <summary>
        /// Dummy ego is used when there is no real Ego in the scene
        /// </summary>
        public void SetDummyEgo(GameObject ego)
        {
            dummyEgo = ego.transform;
        }

        /// <summary>
        /// Registers Ego vehicle.
        /// </summary>
        public void RegisterEgo(GameObject egoVehicle)
        {
            EGOVehicle = egoVehicle.transform;
        }

        /// <summary>
        /// Register <see cref="NPCVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="lane">Initial lane of the vehicle</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(NPCVehicle vehicle, TrafficLane lane, int waypointIndex)
        {
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, lane, waypointIndex));
        }

        /// <summary>
        /// Register <see cref="NPCVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="route">Route for vehicle to follow</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(NPCVehicle vehicle, List<TrafficLane> route, int waypointIndex)
        {
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, route, waypointIndex));
        }

        /// <summary>
        /// This should be called every time any vehicle is destroyed.
        /// </summary>
        public void RemoveInvalidVehicles()
        {
            (VehicleStates as List<NPCVehicleInternalState>)?.RemoveAll(IsVehicleNull);

            // check if vehicle is null
            bool IsVehicleNull(NPCVehicleInternalState state)
            {
                return state.Vehicle == null;
            }
        }

        /// <summary>
        /// Execute simulation steps and update visualization.
        /// </summary>
        /// <param name="deltaTime">Simulation time step</param>
        public void StepOnce(float deltaTime)
        {
            // Simulation steps
            Profiler.BeginSample("NPCVehicleSimulator.Cognition");
            cognitionStep.Execute(vehicleStates, EGOVehicle);
            Profiler.EndSample();

            simulationTime += deltaTime;
            ApplyForcedPass();

            Profiler.BeginSample("NPCVehicleSimulator.Decision");
            decisionStep.Execute(VehicleStates);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Control");
            controlStep.Execute(VehicleStates, deltaTime);
            Profiler.EndSample();

            UpdateDeadlockAndStuck(deltaTime);

            // Visualization step
            Profiler.BeginSample("NPCVehicleSimulator.Visualize");
            visualizationStep.Execute(VehicleStates, EGOVehicle);
            Profiler.EndSample();
        }

        /// <summary>
        /// Show editor gizmos for debugging.
        /// </summary>
        static bool IsWaitingForOthers(NPCVehicleYieldPhase phase)
            => phase != NPCVehicleYieldPhase.NONE && phase != NPCVehicleYieldPhase.ENTERING_INTERSECTION &&
               phase != NPCVehicleYieldPhase.AT_INTERSECTION;

        /// <summary>A vehicle that was given the right of way does not yield (it still stops for lights and vehicles ahead).</summary>
        private void ApplyForcedPass()
        {
            foreach (var state in vehicleStates)
                if (!state.ShouldDespawn && state.ForcedPassUntil > simulationTime && IsWaitingForOthers(state.YieldPhase))
                    state.YieldPhase = NPCVehicleYieldPhase.NONE;
        }

        /// <summary>
        /// Breaks right-of-way deadlocks (two vehicles yielding to each other forever) and removes vehicles stuck for a long time.
        /// </summary>
        private void UpdateDeadlockAndStuck(float deltaTime)
        {
            var b = Behaviour;
            List<NPCVehicleInternalState> candidates = null;
            foreach (var state in vehicleStates)
            {
                if (state.ShouldDespawn || state.Vehicle == null)
                    continue;
                bool still = state.Speed < b.standstillSpeed;
                state.StuckTime = still ? state.StuckTime + deltaTime : 0f;
                state.YieldWaitTime = still && IsWaitingForOthers(state.YieldPhase) ? state.YieldWaitTime + deltaTime : 0f;

                if (b.stuckDespawnTimeout > 0f && state.StuckTime > b.stuckDespawnTimeout)
                {
                    state.ShouldDespawn = true;
                    StuckDespawnCount++;
                    var lane = state.FollowingLanes.Count > 0 && state.FollowingLanes[0] != null ? state.FollowingLanes[0].name : "-";
                    Debug.Log($"[Traffic] removed {state.Vehicle.name}: standing still for {state.StuckTime:F0} s on {lane} " +
                              $"(yield {state.YieldPhase}, light {(state.TrafficLightLane != null ? state.TrafficLightPassability.ToString() : "-")}, " +
                              $"front vehicle {(state.DistanceToFrontVehicle < 1000f ? state.DistanceToFrontVehicle.ToString("F1") + " m" : "-")}, " +
                              $"zebra {state.IsZebraBlocked}, speed mode {state.SpeedMode})");
                    continue;
                }
                if (b.deadlockTimeout > 0f && state.YieldWaitTime > b.deadlockTimeout && state.ForcedPassUntil <= simulationTime)
                    (candidates ??= new List<NPCVehicleInternalState>()).Add(state);
            }
            if (candidates == null)
                return;

            // Longest wait first, then the lower vehicle ID: deterministic, and only one winner per area.
            candidates.Sort((x, y) => x.YieldWaitTime != y.YieldWaitTime
                ? y.YieldWaitTime.CompareTo(x.YieldWaitTime) : x.Vehicle.VehicleID.CompareTo(y.Vehicle.VehicleID));
            var radiusSq = b.forcedPassRadius * b.forcedPassRadius;
            foreach (var c in candidates)
            {
                bool areaBusy = false;
                foreach (var other in vehicleStates)
                    if (other != c && other.ForcedPassUntil > simulationTime && (other.Position - c.Position).sqrMagnitude < radiusSq)
                    {
                        areaBusy = true;
                        break;
                    }
                if (areaBusy)
                    continue;
                c.ForcedPassUntil = simulationTime + b.forcedPassDuration;
                c.YieldWaitTime = 0f;
                ForcedPassCount++;
            }
        }

        public void ShowGizmos(bool showYieldingPhase, bool showObstacleChecking)
        {
            decisionStep.ShowGizmos(VehicleStates);
            cognitionStep.ShowGizmos(VehicleStates, showYieldingPhase, showObstacleChecking);
        }

        public void ClearAll()
        {
            foreach (var state in VehicleStates)
            {
                state.ShouldDespawn = true;
            }
        }

        /// <summary>
        /// Mark vehicles within a specified radius from a transform's position for despawning.
        /// </summary>
        /// <param name="centerPosition">The transform whose position to check from</param>
        /// <param name="radius">The radius to check within</param>
        /// <returns>Number of vehicles marked for despawn</returns>
        public int RemoveVehiclesInRadius(Vector3 centerPosition, float radius)
        {
            if (centerPosition == null)
            {
                Debug.LogWarning("RemoveVehiclesInRadius: centerTransform is null");
                return 0;
            }


            int removedCount = 0;
            float radiusSquared = radius * radius; // Use squared distance for performance

            foreach (var state in vehicleStates)
            {
                if (state.ShouldDespawn)
                    continue;

                // Check distance using the vehicle's position
                var distanceSquared = (state.Position - centerPosition).sqrMagnitude;

                if (distanceSquared <= radiusSquared)
                {
                    state.ShouldDespawn = true;
                    removedCount++;
                }
            }

            return removedCount;
        }

        public void Dispose()
        {
            cognitionStep?.Dispose();
        }
    }
}