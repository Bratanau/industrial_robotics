using UnityEngine;

/// <summary>
/// Simulates a YOLO-based ball detector without actually rendering a camera image.
/// Instead, it projects the target ball's 3D world position into the camera's
/// viewport and derives the same kind of output a real YOLO bounding-box
/// detector would give: a horizontal angle offset and a normalized distance,
/// plus a visibility flag.
///
/// Attach this to the robot's camera GameObject (needs a Camera component).
/// </summary>
[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("The ball to detect. Can be left empty and set at runtime.")]
    public Transform targetBall;

    [Header("Field of view / range")]
    [Tooltip("Horizontal field of view of the simulated camera, degrees")]
    public float hFov = 40f;

    [Tooltip("Maximum distance at which the ball can be seen, meters")]
    public float maxViewDistance = 2f;

    [Header("Line of sight")]
    [Tooltip("Layers considered as walls/obstacles that block the camera's view")]
    public LayerMask obstacleMask = ~0;

    private Camera cam;

    // --- Public read-only detection results, updated each frame ---

    /// <summary>True if the ball is currently visible to the camera.</summary>
    public bool IsBallVisible { get; private set; }

    /// <summary>
    /// Horizontal offset from the center of the frame, roughly in the
    /// range -1 (left edge) .. 0 (center) .. +1 (right edge).
    /// Only meaningful when IsBallVisible is true.
    /// </summary>
    public float HorizontalOffset { get; private set; }

    /// <summary>
    /// Normalized distance to the ball: 0 = right next to the camera,
    /// 1 = at or beyond maxViewDistance. Only meaningful when IsBallVisible is true.
    /// </summary>
    public float NormalizedDistance { get; private set; }

    private void Awake()
    {
        cam = GetComponent<Camera>();
    }

    private void Update()
    {
        UpdateDetection();
    }

    private void UpdateDetection()
    {
        if (targetBall == null || cam == null)
        {
            IsBallVisible = false;
            HorizontalOffset = 0f;
            NormalizedDistance = 1f;
            return;
        }

        Vector3 toBall = targetBall.position - cam.transform.position;
        float distance = toBall.magnitude;

        // 1) Range check.
        if (distance > maxViewDistance)
        {
            SetNotVisible();
            return;
        }

        // 2) Horizontal FOV check (angle between camera forward and direction to ball,
        //    measured only on the horizontal plane so vertical offset doesn't matter).
        Vector3 flatForward = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up);
        Vector3 flatToBall = Vector3.ProjectOnPlane(toBall, Vector3.up);
        float horizontalAngle = Vector3.SignedAngle(flatForward, flatToBall, Vector3.up);

        if (Mathf.Abs(horizontalAngle) > hFov * 0.5f)
        {
            SetNotVisible();
            return;
        }

        // 3) Line-of-sight check: nothing solid between camera and ball.
        if (Physics.Raycast(cam.transform.position, toBall.normalized, out RaycastHit hit, distance, obstacleMask))
        {
            if (hit.transform != targetBall)
            {
                // Something else (a wall) is blocking the view.
                SetNotVisible();
                return;
            }
        }

        // 4) Ball is visible - compute YOLO-like output via viewport projection.
        Vector3 viewportPoint = cam.WorldToViewportPoint(targetBall.position);

        // viewportPoint.x is 0..1 across the frame; remap to -1..+1 (left..right).
        float horizontalOffset = (viewportPoint.x - 0.5f) * 2f;

        // Normalized distance: 0 = close, 1 = at max range.
        float normalizedDistance = Mathf.Clamp01(distance / maxViewDistance);

        IsBallVisible = true;
        HorizontalOffset = Mathf.Clamp(horizontalOffset, -1f, 1f);
        NormalizedDistance = normalizedDistance;
    }

    private void SetNotVisible()
    {
        IsBallVisible = false;
        HorizontalOffset = 0f;
        NormalizedDistance = 1f;
    }

    private void OnDrawGizmosSelected()
    {
        // Visualize the horizontal FOV cone in the Scene view for easier tuning.
        if (cam == null) cam = GetComponent<Camera>();
        if (cam == null) return;

        Gizmos.color = Color.yellow;
        Vector3 origin = cam.transform.position;
        Quaternion leftRot = Quaternion.AngleAxis(-hFov * 0.5f, Vector3.up);
        Quaternion rightRot = Quaternion.AngleAxis(hFov * 0.5f, Vector3.up);

        Vector3 leftDir = leftRot * cam.transform.forward;
        Vector3 rightDir = rightRot * cam.transform.forward;

        Gizmos.DrawLine(origin, origin + leftDir * maxViewDistance);
        Gizmos.DrawLine(origin, origin + rightDir * maxViewDistance);
        Gizmos.DrawLine(origin, origin + cam.transform.forward * maxViewDistance);
    }
}
