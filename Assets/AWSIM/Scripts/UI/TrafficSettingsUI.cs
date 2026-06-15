using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System;
using UnityEngine;
using UnityEngine.UI;
using AWSIM.TrafficSimulation;

namespace AWSIM
{
    public class TrafficSettingsUI : MonoBehaviour
    {
        [SerializeField] TrafficManager trafficManager;
        [SerializeField] InputField trafficSpawnPathInputField;
        [SerializeField] InputField pedestrianPathInputField;
        [SerializeField] GameObject appliedTextObj;

        void Start()
        {
            if (trafficManager != null && trafficSpawnPathInputField != null)
            {
                trafficSpawnPathInputField.text = trafficManager.GetSpawnConfigPath();
            }
        }

        /// <summary>Called by the Canvas prefab button (legacy name).</summary>
        public void RestartRandomTraffic() => ApplyPaths();

        public void ApplyPaths()
        {
            if (trafficManager != null && trafficSpawnPathInputField != null && !string.IsNullOrEmpty(trafficSpawnPathInputField.text))
            {
                trafficManager.SetSpawnConfigPath(trafficSpawnPathInputField.text);
            }

            if (pedestrianPathInputField != null && !string.IsNullOrEmpty(pedestrianPathInputField.text))
            {
                ApplyPedestrianPath(pedestrianPathInputField.text);
            }

            StartCoroutine(DisplayAppliedText());
        }

        private void ApplyPedestrianPath(string path)
        {
            if (string.IsNullOrEmpty(path))
                return;

            var loader = FindPathConfigLoader();
            if (loader == null)
            {
                Debug.LogWarning("[TrafficSettingsUI] PathConfigLoader not found in scene; cannot apply pedestrian path.");
                return;
            }

            var type = loader.GetType();
            var setPath = type.GetMethod("SetJsonConfigPath");
            var loadPaths = type.GetMethod("LoadPaths");
            if (setPath != null)
                setPath.Invoke(loader, new object[] { path });
            if (loadPaths != null)
                loadPaths.Invoke(loader, null);
        }

        private UnityEngine.Object FindPathConfigLoader()
        {
            // Look for any MonoBehaviour with type name "PathConfigLoader"
            var all = Resources.FindObjectsOfTypeAll<MonoBehaviour>();
            return all.FirstOrDefault(m => m != null && m.GetType().Name == "PathConfigLoader");
        }

        IEnumerator DisplayAppliedText()
        {
            appliedTextObj.SetActive(true);
            yield return new WaitForSecondsRealtime(3f);
            appliedTextObj.SetActive(false);
        }
    }
}
