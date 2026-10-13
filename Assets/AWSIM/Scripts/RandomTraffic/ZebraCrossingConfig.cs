using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Optional parent config that provides allowed occupants for all child zebra areas.
    /// Place this on a common parent and zebra children will inherit these lists.
    /// </summary>
    public class ZebraCrossingConfig : MonoBehaviour
    {
        [SerializeField, Tooltip("Whitelist: only count occupants whose root GameObject matches one of these.")]
        private List<GameObject> allowedPrefabs = new List<GameObject>();
        [SerializeField, Tooltip("Whitelist (by name substring) for occupant root GameObjects (case-insensitive).")]
        private List<string> allowedRootNameSubstrings = new List<string>();

        public IReadOnlyList<GameObject> AllowedPrefabs => allowedPrefabs;
        public IReadOnlyList<string> AllowedRootNameSubstrings => allowedRootNameSubstrings;
    }
}
