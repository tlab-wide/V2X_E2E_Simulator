using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class DemoUICustom : MonoBehaviour
    {
        [SerializeField] Text timeScaleText;
        [SerializeField] Slider timeScaleSlider;
        [SerializeField] Text versionText;

        private void Start()
        {
            timeScaleSlider.value = timeScaleSlider.value;
            timeScaleText.text = "x " + timeScaleSlider.value.ToString("F2");
            var version = Application.version;
            print(version);
            versionText.text = "AWSIM v " + version;
        }

        public void SetTimeScale(float timeScale)
        {
            Time.timeScale = timeScale;
            timeScaleText.text = "x " + timeScale.ToString("F2");
        }
    }
}