using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Simple free-fly camera controller for the movable "main" camera view.
    /// Hold Right Mouse Button to look around, WASD to move, Q/E for down/up,
    /// LeftShift to move faster, scroll wheel to dolly forward/backward.
    ///
    /// This behaviour is enabled only while its camera is the active view
    /// (see <see cref="CameraViewManager"/>). It uses unscaled time so the
    /// camera keeps responding even when the simulation time scale changes.
    /// </summary>
    public class FreeFlyCamera : MonoBehaviour
    {
        [Header("Movement")]
        [Tooltip("Base movement speed in meters per second.")]
        public float MoveSpeed = 15.0f;

        [Tooltip("Multiplier applied to movement speed while Left Shift is held.")]
        public float SprintMultiplier = 3.0f;

        [Tooltip("Dolly amount applied per mouse scroll wheel notch.")]
        public float ScrollSpeed = 40.0f;

        [Header("Look")]
        [Tooltip("Mouse look sensitivity (only active while Right Mouse Button is held).")]
        public float LookSensitivity = 3.0f;

        [Tooltip("Minimum / maximum pitch angle in degrees.")]
        public float MinPitch = -89.0f;
        public float MaxPitch = 89.0f;

        private float yaw;
        private float pitch;

        void OnEnable()
        {
            // Sync internal angles with current orientation so we don't snap on activation.
            Vector3 e = transform.eulerAngles;
            yaw = e.y;
            pitch = NormalizePitch(e.x);
        }

        void Update()
        {
            float dt = Time.unscaledDeltaTime;

            // --- Look (right mouse button held) ---
            if (Input.GetMouseButton(1))
            {
                yaw += Input.GetAxis("Mouse X") * LookSensitivity;
                pitch -= Input.GetAxis("Mouse Y") * LookSensitivity;
                pitch = Mathf.Clamp(pitch, MinPitch, MaxPitch);
                transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
            }

            // --- Move (WASD + Q/E) ---
            Vector3 dir = Vector3.zero;
            if (Input.GetKey(KeyCode.W)) dir += Vector3.forward;
            if (Input.GetKey(KeyCode.S)) dir += Vector3.back;
            if (Input.GetKey(KeyCode.A)) dir += Vector3.left;
            if (Input.GetKey(KeyCode.D)) dir += Vector3.right;
            if (Input.GetKey(KeyCode.E)) dir += Vector3.up;
            if (Input.GetKey(KeyCode.Q)) dir += Vector3.down;

            float speed = MoveSpeed * (Input.GetKey(KeyCode.LeftShift) ? SprintMultiplier : 1.0f);
            if (dir.sqrMagnitude > 0.0001f)
            {
                transform.Translate(dir.normalized * speed * dt, Space.Self);
            }

            // --- Dolly (scroll wheel) ---
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (Mathf.Abs(scroll) > 0.0001f)
            {
                transform.Translate(Vector3.forward * scroll * ScrollSpeed, Space.Self);
            }
        }

        private static float NormalizePitch(float x)
        {
            // Convert 0..360 euler back to a signed -180..180 value.
            if (x > 180f) x -= 360f;
            return x;
        }
    }
}
