using UnityEngine;

namespace V2X.Paths
{
    /// <summary>Metadata attached to generated path GameObjects to allow editing type/spawn rate.</summary>
    /// <remarks>In its own file so Unity can find the script when path objects are saved in a scene.</remarks>
    public class PathMetadata : MonoBehaviour
    {
        public PathConfigLoader.PathUserType type;
        public float spawnPerMinute = 1f;
    }
}
