using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Manages multiple camera views and lets the user switch between them from a
    /// runtime-built "View" tab in the canvas.
    ///
    /// Typical setup (3 views):
    ///   0. Follow camera        -> the existing "Main Camera (1)" (FollowCamera script)
    ///   1. Top view (fixed)     -> the existing "Camera Top View Int#1"
    ///   2. Movable / free camera -> auto-created here (or assigned), driven by FreeFlyCamera
    ///
    /// How switching works: exactly one view is active at a time. For the active view
    /// its Camera + AudioListener (and optional control behaviour, e.g. FreeFlyCamera)
    /// are enabled; every other view has them disabled.
    ///
    /// Just drop this component on a single GameObject, assign the cameras, and press play.
    /// The "View" tab UI is created automatically (its own Screen-Space-Overlay canvas
    /// unless you assign an existing one).
    /// </summary>
    public class CameraViewManager : MonoBehaviour
    {
        #region [Data]

        [Serializable]
        public class CameraView
        {
            [Tooltip("Label shown on the button in the View tab.")]
            public string Label = "View";

            [Tooltip("Camera that renders when this view is selected.")]
            public Camera Camera;

            [Tooltip("Optional behaviour enabled only while this view is active " +
                     "(e.g. FollowCamera or FreeFlyCamera). Leave empty if not needed.")]
            public MonoBehaviour ControlBehaviour;
        }

        #endregion

        #region [Inspector]

        [Header("Camera Views (in tab order)")]
        [Tooltip("The cameras to switch between. The first entry is the default active view.")]
        [SerializeField] private List<CameraView> views = new List<CameraView>();

        [Header("Movable Free Camera")]
        [Tooltip("If enabled and no movable camera is assigned below, one is created at runtime " +
                 "(starting from the first view's pose) with a FreeFlyCamera controller.")]
        [SerializeField] private bool autoCreateMovableCamera = true;

        [Tooltip("Label used for the auto-created movable camera view.")]
        [SerializeField] private string movableCameraLabel = "Free Camera";

        [Tooltip("Optional: assign an existing camera to use as the movable camera instead of auto-creating one.")]
        [SerializeField] private Camera movableCamera;

        [Header("UI")]
        [Tooltip("Build the 'View' tab UI automatically at runtime. " +
                 "Turn OFF when the View panel already exists in the scene (buttons wired to SelectView).")]
        [SerializeField] private bool buildRuntimeUI = true;

        [Tooltip("Optional: parent canvas to attach the View tab to. If empty, a dedicated overlay canvas is created.")]
        [SerializeField] private Canvas targetCanvas;

        [Header("Pre-built UI (used when 'Build Runtime UI' is OFF)")]
        [Tooltip("The View panel GameObject that is shown / hidden with the toggle key.")]
        [SerializeField] private GameObject uiPanel;

        [Tooltip("Buttons of the View panel, in the SAME order as the camera views. " +
                 "Used to highlight the active view. Wire each button's OnClick to SelectView(index).")]
        [SerializeField] private List<Button> uiButtons = new List<Button>();

        [Tooltip("Key that shows/hides the View tab panel.")]
        [SerializeField] private KeyCode togglePanelKey = KeyCode.V;

        [Tooltip("Start with the View panel expanded.")]
        [SerializeField] private bool startExpanded = true;

        #endregion

        #region [Private]

        private int activeIndex = -1;
        private readonly List<Button> viewButtons = new List<Button>();
        private GameObject panelRoot;
        private bool expanded;
        private bool runtimeBuilt;

        private static readonly Color ActiveColor = new Color(0.16f, 0.5f, 0.85f, 1f);
        private static readonly Color InactiveColor = new Color(0.20f, 0.20f, 0.22f, 0.9f);

        #endregion

        #region [Unity]

        void Start()
        {
            SetupMovableCamera();

            if (views.Count == 0)
            {
                Debug.LogWarning($"{nameof(CameraViewManager)}: no camera views assigned. Assign cameras in the inspector.");
                return;
            }

            if (buildRuntimeUI)
                BuildUI();
            else
                BindExternalUI();

            expanded = startExpanded;
            ApplyPanelVisibility();

            // Default to the first view.
            SelectView(0);
        }

        void Update()
        {
            if (Input.GetKeyDown(togglePanelKey))
            {
                expanded = !expanded;
                ApplyPanelVisibility();
            }

            // Number-key shortcuts 1..9 to switch views.
            for (int i = 0; i < views.Count && i < 9; i++)
            {
                if (Input.GetKeyDown(KeyCode.Alpha1 + i))
                    SelectView(i);
            }
        }

        #endregion

        #region [Public API]

        /// <summary>
        /// Activate the view at <paramref name="index"/>: enable its camera / audio / control
        /// and disable every other view's.
        /// </summary>
        public void SelectView(int index)
        {
            if (index < 0 || index >= views.Count)
                return;

            activeIndex = index;

            for (int i = 0; i < views.Count; i++)
            {
                bool isActive = (i == index);
                ApplyViewActive(views[i], isActive);
                UpdateButtonVisual(i, isActive);
            }
        }

        #endregion

        #region [Camera switching]

        private void ApplyViewActive(CameraView view, bool isActive)
        {
            if (view == null || view.Camera == null)
                return;

            // Make sure the camera GameObject itself is active so it can render / be moved.
            if (view.Camera.gameObject.activeSelf != isActive)
                view.Camera.gameObject.SetActive(isActive);

            view.Camera.enabled = isActive;

            var listener = view.Camera.GetComponent<AudioListener>();
            if (listener != null)
                listener.enabled = isActive;

            if (view.ControlBehaviour != null)
                view.ControlBehaviour.enabled = isActive;
        }

        private void SetupMovableCamera()
        {
            // Assigned explicitly -> just make sure it's in the views list.
            if (movableCamera != null)
            {
                AddMovableToViews(movableCamera);
                return;
            }

            if (!autoCreateMovableCamera)
                return;

            // Seed pose from the first assigned view's camera (usually the follow camera).
            Vector3 pos = Vector3.up * 10f;
            Quaternion rot = Quaternion.identity;
            if (views.Count > 0 && views[0].Camera != null)
            {
                pos = views[0].Camera.transform.position;
                rot = views[0].Camera.transform.rotation;
            }

            var go = new GameObject("Movable Camera");
            go.transform.SetPositionAndRotation(pos, rot);

            var cam = go.AddComponent<Camera>();
            // Copy render settings from the follow camera when available so it looks consistent.
            if (views.Count > 0 && views[0].Camera != null)
                cam.CopyFrom(views[0].Camera);

            var listener = go.AddComponent<AudioListener>();
            listener.enabled = false;

            var fly = go.AddComponent<FreeFlyCamera>();
            fly.enabled = false;

            cam.enabled = false;
            go.SetActive(false);

            movableCamera = cam;

            views.Add(new CameraView
            {
                Label = movableCameraLabel,
                Camera = cam,
                ControlBehaviour = fly,
            });
        }

        private void AddMovableToViews(Camera cam)
        {
            // Avoid duplicates if the user already added it to the list.
            foreach (var v in views)
            {
                if (v.Camera == cam)
                    return;
            }

            views.Add(new CameraView
            {
                Label = movableCameraLabel,
                Camera = cam,
                ControlBehaviour = cam.GetComponent<FreeFlyCamera>(),
            });
        }

        #endregion

        #region [UI building]

        /// <summary>
        /// Bind to a View panel that already exists in the scene (buttons wired to SelectView).
        /// </summary>
        private void BindExternalUI()
        {
            runtimeBuilt = false;
            panelRoot = uiPanel;

            viewButtons.Clear();
            if (uiButtons != null)
                viewButtons.AddRange(uiButtons);
        }

        private void BuildUI()
        {
            runtimeBuilt = true;
            EnsureEventSystem();

            Transform parent;
            if (targetCanvas != null)
            {
                parent = targetCanvas.transform;
            }
            else
            {
                var canvasGo = new GameObject("ViewTabCanvas");
                var canvas = canvasGo.AddComponent<Canvas>();
                canvas.renderMode = RenderMode.ScreenSpaceOverlay;
                canvas.sortingOrder = 100;
                var scaler = canvasGo.AddComponent<CanvasScaler>();
                scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
                scaler.referenceResolution = new Vector2(1920, 1080);
                canvasGo.AddComponent<GraphicRaycaster>();
                parent = canvasGo.transform;
            }

            // Root container anchored to the top-left.
            panelRoot = new GameObject("ViewTab", typeof(RectTransform));
            var rootRt = (RectTransform)panelRoot.transform;
            rootRt.SetParent(parent, false);
            rootRt.anchorMin = new Vector2(0f, 1f);
            rootRt.anchorMax = new Vector2(0f, 1f);
            rootRt.pivot = new Vector2(0f, 1f);
            rootRt.anchoredPosition = new Vector2(16f, -16f);
            rootRt.sizeDelta = new Vector2(170f, 10f);

            var layout = panelRoot.AddComponent<VerticalLayoutGroup>();
            layout.childControlWidth = true;
            layout.childControlHeight = true;
            layout.childForceExpandWidth = true;
            layout.childForceExpandHeight = false;
            layout.spacing = 4f;
            layout.padding = new RectOffset(6, 6, 6, 6);

            var fitter = panelRoot.AddComponent<ContentSizeFitter>();
            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            var bg = panelRoot.AddComponent<Image>();
            bg.color = new Color(0f, 0f, 0f, 0.55f);

            // Header row: a "View" tab button that toggles the list.
            var header = CreateButton(panelRoot.transform, "▾ View");
            header.onClick.AddListener(() =>
            {
                expanded = !expanded;
                ApplyPanelVisibility();
            });
            SetButtonColor(header, new Color(0.12f, 0.12f, 0.14f, 1f));

            // One button per view.
            viewButtons.Clear();
            for (int i = 0; i < views.Count; i++)
            {
                int captured = i;
                var label = string.IsNullOrEmpty(views[i].Label) ? $"View {i + 1}" : views[i].Label;
                var btn = CreateButton(panelRoot.transform, $"{i + 1}. {label}");
                btn.onClick.AddListener(() => SelectView(captured));
                viewButtons.Add(btn);
            }
        }

        private void ApplyPanelVisibility()
        {
            if (panelRoot == null)
                return;

            if (!runtimeBuilt)
            {
                // Pre-built panel: simply show / hide the whole panel.
                panelRoot.SetActive(expanded);
                return;
            }

            // Runtime-built panel: keep the header visible, only toggle the view buttons.
            foreach (var btn in viewButtons)
            {
                if (btn != null)
                    btn.gameObject.SetActive(expanded);
            }

            // Update the header caption arrow.
            var headerText = panelRoot.transform.GetChild(0).GetComponentInChildren<Text>();
            if (headerText != null)
                headerText.text = expanded ? "▾ View" : "▸ View";
        }

        private void UpdateButtonVisual(int index, bool isActive)
        {
            if (index < 0 || index >= viewButtons.Count)
                return;
            SetButtonColor(viewButtons[index], isActive ? ActiveColor : InactiveColor);
        }

        private static Button CreateButton(Transform parent, string caption)
        {
            var go = new GameObject("Button", typeof(RectTransform));
            go.transform.SetParent(parent, false);

            var image = go.AddComponent<Image>();
            image.color = InactiveColor;

            var button = go.AddComponent<Button>();
            button.targetGraphic = image;

            var le = go.AddComponent<LayoutElement>();
            le.minHeight = 30f;
            le.preferredHeight = 30f;

            var textGo = new GameObject("Text", typeof(RectTransform));
            textGo.transform.SetParent(go.transform, false);
            var textRt = (RectTransform)textGo.transform;
            textRt.anchorMin = Vector2.zero;
            textRt.anchorMax = Vector2.one;
            textRt.offsetMin = new Vector2(8f, 0f);
            textRt.offsetMax = new Vector2(-8f, 0f);

            var text = textGo.AddComponent<Text>();
            text.text = caption;
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf")
                        ?? Resources.GetBuiltinResource<Font>("Arial.ttf");
            text.fontSize = 15;
            text.alignment = TextAnchor.MiddleLeft;
            text.color = Color.white;

            return button;
        }

        private static void SetButtonColor(Button button, Color color)
        {
            if (button == null)
                return;
            var image = button.targetGraphic as Image;
            if (image != null)
                image.color = color;
        }

        private static void EnsureEventSystem()
        {
            if (FindObjectOfType<EventSystem>() == null)
            {
                var es = new GameObject("EventSystem");
                es.AddComponent<EventSystem>();
                es.AddComponent<StandaloneInputModule>();
            }
        }

        #endregion
    }
}
