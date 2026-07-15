using UnityEngine;
using UnityEngine.InputSystem;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// ML-Agents brain for the robot. Collects a 15-value observation vector
/// from the robot's subsystems (ultrasonic/IR sensors, simulated YOLO camera,
/// gripper, drivetrain) and turns received actions into commands for
/// TrackController, the camera servo, and GripperController.
///
/// Attach to the robot's root GameObject alongside TrackController,
/// VirtualSensors, GripperController and (indirectly) SimulatedYoloCamera.
/// </summary>
public class RobotBrain : Agent
{
    [Header("Subsystem references")]
    public TrackController trackController;
    public VirtualSensors sensors;
    public GripperController gripper;
    public SimulatedYoloCamera yoloCamera;
    public Rigidbody rb;

    [Header("Camera servo")]
    [Tooltip("Transform that physically rotates to pan the camera (child of the robot)")]
    public Transform cameraServoPivot;
    [Tooltip("Max servo angle in either direction from center, degrees")]
    public float maxServoAngle = 60f;
    [Tooltip("Servo rotation speed, degrees/second, at full action signal")]
    public float servoSpeed = 90f;

    [Header("Observation helpers")]
    [Tooltip("Heading angle is normalized by dividing by this value (degrees)")]
    public float headingNormalizer = 180f;
    [Tooltip("Time-since-detection observation is normalized by dividing by this value (seconds)")]
    public float timeSinceDetectionNormalizer = 5f;

    [Header("Reward tuning")]
    [Tooltip("Reward scale for closing distance to the ball while still far away")]
    public float distanceRewardScaleFar = 1f;
    [Tooltip("Reward scale for closing distance to the ball once already close (rewards fine approach more)")]
    public float distanceRewardScaleNear = 3f;
    [Tooltip("World distance (meters) below which the robot is considered 'close' to the ball")]
    public float closeDistanceThreshold = 0.5f;

    [Tooltip("Penalty scale for changing gas/steer sharply between steps")]
    public float actionRatePenaltyScale = 0.05f;

    [Tooltip("Reward scale for facing the ball (based on camera horizontal offset)")]
    public float centeringRewardScale = 0.02f;

    [Tooltip("Ultrasonic normalized distance (0..1) below which the robot is 'too close' to a wall")]
    public float wallProximityThreshold = 0.15f;
    [Tooltip("Penalty applied per step while too close to a wall")]
    public float wallPenaltyScale = 0.05f;
    [Tooltip("Extra penalty applied for each short-range IR sensor currently triggered")]
    public float irWallPenalty = 0.02f;

    [Tooltip("Terminal reward for successfully grabbing and holding the ball")]
    public float successReward = 5f;
    [Tooltip("Terminal penalty for falling off the arena")]
    public float fallPenalty = -1f;

    [Header("Arena bounds")]
    [Tooltip("Collider representing the playable arena floor. If the robot leaves these bounds, the episode ends with a penalty. Optional if using fallYThreshold instead.")]
    public Collider arenaBounds;
    [Tooltip("Fallback: if the robot's Y position drops below this, it is considered to have fallen off the arena")]
    public float fallYThreshold = -1f;

    // --- Internal state ---
    private Vector3 startPosition;
    private float currentServoAngle;      // degrees, -maxServoAngle .. +maxServoAngle
    private float lastKnownBallDirection;  // last non-zero horizontal offset seen
    private float timeSinceLastDetection;
    private float previousDistanceToBall;
    private float previousLinearAction;
    private float previousAngularAction;

    public override void Initialize()
    {
        if (trackController == null) trackController = GetComponent<TrackController>();
        if (sensors == null) sensors = GetComponent<VirtualSensors>();
        if (gripper == null) gripper = GetComponent<GripperController>();
        if (rb == null) rb = GetComponent<Rigidbody>();

        startPosition = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        startPosition = transform.position;
        currentServoAngle = 0f;
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousLinearAction = 0f;
        previousAngularAction = 0f;

        previousDistanceToBall = GetDistanceToBall();

        if (cameraServoPivot != null)
        {
            cameraServoPivot.localRotation = Quaternion.identity;
        }
    }

    private void Update()
    {
        // Track "time since last ball detection" and "last known direction"
        // continuously, independently of the ML-Agents decision cadence.
        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            lastKnownBallDirection = yoloCamera.HorizontalOffset;
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.deltaTime;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. Normalized ultrasonic distance (0 = touching, 1 = clear).
        sensor.AddObservation(sensors != null ? sensors.UltrasonicDistance01 : 1f);

        // 2. Left IR obstacle sensor (0/1).
        sensor.AddObservation(sensors != null ? sensors.LeftIR : 0);

        // 3. Right IR obstacle sensor (0/1).
        sensor.AddObservation(sensors != null ? sensors.RightIR : 0);

        // 4. Gripper IR sensor (0/1).
        sensor.AddObservation(sensors != null ? sensors.GripperIR : 0);

        // 5. Relative horizontal angle to the ball from the camera (0 if not visible).
        bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible;
        sensor.AddObservation(ballVisible ? yoloCamera.HorizontalOffset : 0f);

        // 6. Normalized distance to the ball from the camera (1 if not visible).
        sensor.AddObservation(ballVisible ? yoloCamera.NormalizedDistance : 1f);

        // 7. Last known direction to the ball (after it left the frame).
        sensor.AddObservation(lastKnownBallDirection);

        // 8. Ball visibility flag (0.0 / 1.0).
        sensor.AddObservation(ballVisible ? 1f : 0f);

        // 9. Current camera servo rotation, normalized to -1..1.
        float normalizedServo = maxServoAngle > 0f ? currentServoAngle / maxServoAngle : 0f;
        sensor.AddObservation(normalizedServo);

        // 10. Gripper hold status (hasBall -> 0.0 / 1.0).
        bool hasBall = gripper != null && gripper.HasBall;
        sensor.AddObservation(hasBall ? 1f : 0f);

        // 11. Relative X offset from the start position.
        Vector3 offset = transform.position - startPosition;
        sensor.AddObservation(offset.x);

        // 12. Relative Z offset from the start position.
        sensor.AddObservation(offset.z);

        // 13. Normalized heading angle of the robot.
        float heading = transform.eulerAngles.y;
        if (heading > 180f) heading -= 360f; // wrap to -180..180
        sensor.AddObservation(heading / headingNormalizer);

        // 14. Current movement speed from the Rigidbody.
        float speed = rb != null ? rb.linearVelocity.magnitude : 0f;
        sensor.AddObservation(speed);

        // 15. Time elapsed since the last ball detection, normalized.
        sensor.AddObservation(timeSinceLastDetection / timeSinceDetectionNormalizer);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // --- Continuous actions ---
        float linear = actions.ContinuousActions[0];   // -1..1
        float angular = actions.ContinuousActions[1];   // -1..1
        float cameraServoSignal = actions.ContinuousActions[2]; // -1..1

        if (trackController != null)
        {
            trackController.SetCommand(linear, angular);
        }

        // Rotate the camera servo pivot based on the signal, clamped to +-maxServoAngle.
        currentServoAngle = Mathf.Clamp(
            currentServoAngle + cameraServoSignal * servoSpeed * Time.deltaTime,
            -maxServoAngle,
            maxServoAngle);

        if (cameraServoPivot != null)
        {
            cameraServoPivot.localRotation = Quaternion.Euler(0f, currentServoAngle, 0f);
        }

        // --- Discrete action: gripper command ---
        // 0 = do nothing, 1 = close, 2 = open.
        int gripperCommand = actions.DiscreteActions[0];

        if (gripper != null)
        {
            if (gripperCommand == 1)
            {
                gripper.SetClawClosed(true);
            }
            else if (gripperCommand == 2)
            {
                gripper.SetClawClosed(false);
            }
            // gripperCommand == 0 -> leave the claw state unchanged.
        }

        CalculateRewards(linear, angular);
    }

    /// <summary>
    /// Computes and applies per-step rewards/penalties, and checks terminal conditions.
    /// Called at the end of OnActionReceived.
    /// </summary>
    private void CalculateRewards(float linear, float angular)
    {
        // 1) Distance-delta reward: reward closing distance to the ball,
        //    with a stronger reward the closer the robot already is.
        float currentDistance = GetDistanceToBall();
        if (currentDistance >= 0f && previousDistanceToBall >= 0f)
        {
            float delta = previousDistanceToBall - currentDistance; // positive = got closer
            bool isClose = currentDistance <= closeDistanceThreshold;
            float scale = isClose ? distanceRewardScaleNear : distanceRewardScaleFar;
            AddReward(delta * scale);
        }
        if (currentDistance >= 0f)
        {
            previousDistanceToBall = currentDistance;
        }

        // 2) Action-rate penalty: discourage jerky gas/steer changes between steps.
        float actionDelta = Mathf.Abs(linear - previousLinearAction) + Mathf.Abs(angular - previousAngularAction);
        AddReward(-actionRatePenaltyScale * actionDelta);
        previousLinearAction = linear;
        previousAngularAction = angular;

        // 3) Centering bonus: reward facing the ball (small horizontal camera offset)
        //    so the robot lines up before attempting a grab.
        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            float alignment = 1f - Mathf.Abs(yoloCamera.HorizontalOffset); // 1 = dead center, 0 = at FOV edge
            AddReward(alignment * centeringRewardScale);
        }

        // 4) Distance-sensor penalty: discourage getting dangerously close to walls.
        if (sensors != null)
        {
            if (sensors.UltrasonicDistance01 < wallProximityThreshold)
            {
                AddReward(-wallPenaltyScale);
            }
            if (sensors.LeftIR == 1) AddReward(-irWallPenalty);
            if (sensors.RightIR == 1) AddReward(-irWallPenalty);
        }

        // 5) Terminal conditions.
        if (gripper != null && gripper.HasBall)
        {
            AddReward(successReward);
            EndEpisode();
            return;
        }

        if (HasFallenOffArena())
        {
            AddReward(fallPenalty);
            EndEpisode();
            return;
        }
    }

    /// <summary>
    /// World-space distance from the robot to the target ball, or -1 if no ball is assigned.
    /// Uses the ball reference from the YOLO camera so both systems agree on the same target.
    /// </summary>
    private float GetDistanceToBall()
    {
        if (yoloCamera == null || yoloCamera.targetBall == null)
        {
            return -1f;
        }
        return Vector3.Distance(transform.position, yoloCamera.targetBall.position);
    }

    /// <summary>
    /// True if the robot is considered to have fallen off the arena, either because it left
    /// the arenaBounds collider (if assigned) or dropped below fallYThreshold.
    /// </summary>
    private bool HasFallenOffArena()
    {
        if (arenaBounds != null)
        {
            return !arenaBounds.bounds.Contains(transform.position);
        }

        return transform.position.y < fallYThreshold;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control fallback for testing without a trained model.
        var continuousActions = actionsOut.ContinuousActions;
        var discreteActions = actionsOut.DiscreteActions;

        var keyboard = Keyboard.current;
        if (keyboard == null)
        {
            continuousActions[0] = 0f;
            continuousActions[1] = 0f;
            continuousActions[2] = 0f;
            discreteActions[0] = 0;
            return;
        }

        float gas = 0f;
        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) gas += 1f;
        if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) gas -= 1f;

        float steer = 0f;
        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) steer += 1f;
        if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) steer -= 1f;

        continuousActions[0] = gas;
        continuousActions[1] = steer;
        continuousActions[2] = 0f;

        if (keyboard.qKey.wasPressedThisFrame) discreteActions[0] = 1; // close
        else if (keyboard.eKey.wasPressedThisFrame) discreteActions[0] = 2; // open
        else discreteActions[0] = 0;
    }
}