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

    [Header("State")]
    [Tooltip("True while the claw is commanded to be closed")]
    public bool isClawClosed;

    /// <summary>True while the gripper is currently holding the ball.</summary>
    public bool HasBall => heldBallRb != null;

    private Rigidbody heldBallRb;
    private Collider heldBallCollider;
    private Transform heldBallOriginalParent;

    /// <summary>
    /// Call this to open/close the claw (e.g. from input or an AI action).
    /// </summary>
    public void SetClawClosed(bool closed)
    {
        isClawClosed = closed;

        if (!closed && heldBallRb != null)
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
    /// Finds the ball currently detected by the gripper IR sensor and switches
    /// it into "logically held" mode: kinematic rigidbody, collider disabled,
    /// parented to HoldPoint.
    /// </summary>
    private void TryGripBall()
    {
        if (holdPoint == null || sensors.gripperIRPoint == null) return;

        if (Physics.Raycast(sensors.gripperIRPoint.position, sensors.gripperIRPoint.forward,
                out RaycastHit hit, sensors.gripperIRRange, sensors.obstacleMask))
        {
            if (!hit.collider.CompareTag(sensors.ballTag)) return;

            heldBallRb = hit.collider.attachedRigidbody;
            heldBallCollider = hit.collider;

            if (heldBallRb == null) return;

            heldBallOriginalParent = heldBallRb.transform.parent;

            heldBallRb.isKinematic = true;
            heldBallCollider.enabled = false;

            heldBallRb.transform.SetParent(holdPoint);
            heldBallRb.transform.localPosition = Vector3.zero;
            heldBallRb.transform.localRotation = Quaternion.identity;
        }
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
}