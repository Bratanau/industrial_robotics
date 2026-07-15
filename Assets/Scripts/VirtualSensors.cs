using UnityEngine;

/// <summary>
/// Simulates the robot's onboard sensors using Physics raycasts:
///  - one ultrasonic sensor (fan of rays, ~30 deg cone, ignores the target ball)
///  - two short-range IR obstacle sensors (left / right, binary output)
///  - one IR sensor inside the gripper (detects the target ball at close range)
/// </summary>
public class VirtualSensors : MonoBehaviour
{
    [Header("Sensor anchor points")]
    public Transform centerPoint;     // faces forward - ultrasonic sensor
    public Transform leftIRPoint;     // faces left - IR obstacle sensor
    public Transform rightIRPoint;    // faces right - IR obstacle sensor
    public Transform gripperIRPoint;  // faces into the claw - IR ball sensor

    [Header("Ultrasonic sensor")]
    [Tooltip("Max detection range, meters")]
    public float ultrasonicRange = 2.0f;
    [Tooltip("Full cone angle, degrees (rays are cast across this fan)")]
    public float ultrasonicConeAngle = 30f;
    [Tooltip("Number of rays cast across the cone")]
    public int ultrasonicRayCount = 5;

    [Header("IR obstacle sensors")]
    [Tooltip("Detection range for short-range wall sensors, meters")]
    public float irObstacleRange = 0.15f;

    [Header("Gripper IR sensor")]
    [Tooltip("Detection range for the ball inside the claw, meters")]
    public float gripperIRRange = 0.08f;

    [Header("Layers / tags")]
    public LayerMask obstacleMask = ~0; // everything by default
    public string ballTag = "TargetBall";

    // --- Public read-only results, updated each frame ---
    public float UltrasonicDistance01 { get; private set; } = 1f; // 0 = touching, 1 = clear
    public int LeftIR { get; private set; }
    public int RightIR { get; private set; }
    public int GripperIR { get; private set; }

    private void Update()
    {
        UltrasonicDistance01 = ReadUltrasonic();
        LeftIR = ReadShortRangeIR(leftIRPoint, irObstacleRange, ignoreBall: false);
        RightIR = ReadShortRangeIR(rightIRPoint, irObstacleRange, ignoreBall: false);
        GripperIR = ReadGripperIR();
    }

    /// <summary>
    /// Casts a fan of rays across the ultrasonic cone, finds the closest hit
    /// (ignoring the ball, since it's too small for real ultrasonic sensors to see),
    /// and returns a normalized distance: 0 = obstacle right at the sensor, 1 = clear.
    /// </summary>
    private float ReadUltrasonic()
    {
        if (centerPoint == null) return 1f;

        float closestDistance = ultrasonicRange;
        bool hitSomething = false;

        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfAngle = ultrasonicConeAngle * 0.5f;

        for (int i = 0; i < rays; i++)
        {
            float t = rays == 1 ? 0f : (float)i / (rays - 1); // 0..1
            float angle = Mathf.Lerp(-halfAngle, halfAngle, t);

            Vector3 direction = Quaternion.Euler(0f, angle, 0f) * centerPoint.forward;

            if (Physics.Raycast(centerPoint.position, direction, out RaycastHit hit, ultrasonicRange, obstacleMask))
            {
                if (hit.collider.CompareTag(ballTag))
                {
                    continue; // ultrasonic can't reliably see the small ball, ignore it
                }

                hitSomething = true;
                if (hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                }
            }
        }

        if (!hitSomething)
        {
            return 1f;
        }

        return Mathf.Clamp01(closestDistance / ultrasonicRange);
    }

    /// <summary>
    /// Single short raycast used for the left/right IR obstacle sensors.
    /// Returns 1 if a wall/obstacle is detected within range, 0 otherwise.
    /// </summary>
    private int ReadShortRangeIR(Transform origin, float range, bool ignoreBall)
    {
        if (origin == null) return 0;

        if (Physics.Raycast(origin.position, origin.forward, out RaycastHit hit, range, obstacleMask))
        {
            if (ignoreBall && hit.collider.CompareTag(ballTag))
            {
                return 0;
            }
            return 1;
        }

        return 0;
    }

    /// <summary>
    /// Detects the target ball at close range inside the gripper.
    /// Returns 1 if the ball is present within gripperIRRange, 0 otherwise.
    /// </summary>
    private int ReadGripperIR()
    {
        if (gripperIRPoint == null) return 0;

        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, gripperIRRange, obstacleMask))
        {
            if (hit.collider.CompareTag(ballTag))
            {
                return 1;
            }
        }

        return 0;
    }
}