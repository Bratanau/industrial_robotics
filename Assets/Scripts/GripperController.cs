using UnityEngine;

/// <summary>
/// Controls the robot's claw/gripper.
/// Since physically holding a round object with rigid jaws is unreliable in
/// a physics simulation (the ball slips or is ejected due to friction solver
/// inaccuracies), this uses a "logical grip": when the gripper IR sensor
/// detects the ball, physics is disabled for it and it is parented to a
/// HoldPoint, so it moves rigidly with the claw. Opening the claw restores
/// normal physics.
/// </summary>
public class GripperController : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Empty transform positioned between the claw's jaws")]
    public Transform holdPoint;

    [Tooltip("VirtualSensors component providing the gripper IR reading")]
    public VirtualSensors sensors;

    [Header("Debug")]
    [Tooltip("Radius used only to draw the gizmo at holdPoint, purely visual")]
    [SerializeField] private float grabSearchRadius = 0.05f;

    [Header("State")]
    [Tooltip("True while the claw is commanded to be closed")]
    public bool isClawClosed;

    /// <summary>True while the gripper is currently holding the ball.</summary>
    public bool IsHolding => heldBallRb != null;

    private Rigidbody heldBallRb;
    private Collider heldBallCollider;
    private Transform heldBallOriginalParent;

    /// <summary>Command the claw to close and attempt to grip the ball.</summary>
    public void GripCommand()
    {
        isClawClosed = true;
    }

    /// <summary>Command the claw to open, releasing the ball if held.</summary>
    public void ReleaseCommand()
    {
        isClawClosed = false;

        if (heldBallRb != null)
        {
            ReleaseBall();
        }
    }

    private void Update()
    {
        if (sensors == null) return;

        // Try to grip when the claw is closed and the gripper IR sees the ball.
        if (isClawClosed && heldBallRb == null && sensors.GripperIR == 1)
        {
            TryGripBall();
        }
    }

    /// <summary>
    /// Uses the ball collider already detected by VirtualSensors' gripper IR
    /// this frame and switches it into "logically held" mode: kinematic
    /// rigidbody, collider disabled, parented to HoldPoint.
    /// </summary>
    private void TryGripBall()
    {
        if (holdPoint == null) return;

        Collider ballCollider = sensors.GripperDetectedBall;
        if (ballCollider == null) return;

        Rigidbody ballRb = ballCollider.attachedRigidbody;
        if (ballRb == null) return;

        heldBallRb = ballRb;
        heldBallCollider = ballCollider;
        heldBallOriginalParent = heldBallRb.transform.parent;

        heldBallRb.isKinematic = true;
        heldBallCollider.enabled = false;

        // worldPositionStays: true сохраняет мировой масштаб мяча,
        // иначе он унаследует масштаб родителя (и раздуется/сожмётся).
        heldBallRb.transform.SetParent(holdPoint, worldPositionStays: true);

        // Затем принудительно телепортируем мяч в позицию HoldPoint.
        heldBallRb.transform.position = holdPoint.position;
        heldBallRb.transform.rotation = holdPoint.rotation;
    }

    /// <summary>
    /// Restores the held ball's physics and re-parents it back to the world.
    /// </summary>
    private void ReleaseBall()
    {
        if (heldBallRb == null) return;

        heldBallRb.transform.SetParent(heldBallOriginalParent);

        heldBallCollider.enabled = true;
        heldBallRb.isKinematic = false;

        heldBallRb = null;
        heldBallCollider = null;
        heldBallOriginalParent = null;
    }

    private void OnDrawGizmosSelected()
    {
        if (holdPoint == null) return;
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(holdPoint.position, grabSearchRadius);
    }
}