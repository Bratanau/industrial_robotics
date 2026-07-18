using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// Interceptor GFS-X AI agent. Glues the team scripts (TrackController,
/// VirtualSensors, SimulatedYoloCamera) into one "brain":
///   - collects 11 observations (order strictly matches the Practice 3 table,
///     minus odometry/heading/speed - the real GFS-X has none of those sensors),
///   - dispatches 3 continuous actions (no discrete action - the claw is autonomous,
///     driven directly by the GripperIR sensor, matching real hardware exactly),
///   - computes bounded rewards (all fixed amounts, no multipliers/percentages),
///   - exposes the public fields that BrainDebugHUD reads.
/// Inherits Agent (ML-Agents). Attach to the "robot" object.
///
/// Behavior Parameters on the robot MUST match:
///   Behavior Name = GFSX_Brain (same as config.yaml)
///   Vector Observation -> Space Size = 11, Stacked Vectors = 4
///   Continuous Actions = 3, Discrete Branches = 0
/// Leave the inspector "Max Step" = 0; this script caps the episode itself.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Robot component references")]
    [SerializeField] private TrackController track;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private SimulatedYoloCamera yolo;
    [Tooltip("Practice 7 (HIL): optional real-YOLO receiver. When assigned AND its useYOLO is ON, vision comes from the real robot instead of the simulated camera")]
    [SerializeField] private RealVision realVision;

    [Header("Domain Randomization (optional)")]
    [Tooltip("Optional DomainRandomizer component. Leave empty to train on a clean environment")]
    [SerializeField] private DomainRandomizer randomizer;
    [Tooltip("Optional ObstacleRandomizer: scatters the obstacle belt every episode (competition rule). Leave empty for a fixed layout")]
    [SerializeField] private ObstacleRandomizer obstacleRandomizer;

    [Header("Camera servo")]
    [Tooltip("Transform rotated by the camera command (usually the camera object / pan joint)")]
    [SerializeField] private Transform cameraServo;
    [SerializeField] private float maxServoAngle = 90f;   // +/-90 deg
    [SerializeField] private float servoSpeed = 90f;      // deg/s

    [Header("Target and arena")]
    [SerializeField] private Transform ball;
    [SerializeField] private string ballTag = "TargetBall";
    [Tooltip("Optional arena center. If empty, world origin (0,0,0) is used")]
    [SerializeField] private Transform arenaCenter;
    [Tooltip("Half-size of the arena in X/Z, meters (4x4 arena => 2.0). Used for the out-of-bounds check")]
    [SerializeField] private float arenaHalf = 2.0f;

    [Header("Ball spawn zone (competition layout)")]
    [Tooltip("Ball spawns in a band AHEAD of the robot's start direction: from minForward to maxForward meters in front of the robot's spawn point")]
    [SerializeField] private float ballMinForward = 2.0f;
    [SerializeField] private float ballMaxForward = 3.2f;
    [Tooltip("Half-width of the ball spawn band sideways from the robot's start axis")]
    [SerializeField] private float ballHalfWidth = 1.5f;
    [Tooltip("Randomize the robot's start position sideways by +- this many meters along its spawn side")]
    [SerializeField] private float robotSpawnJitter = 0.5f;
    [Tooltip("How far below the arena center Y counts as 'fell through the floor'")]
    [SerializeField] private float fallDrop = 0.5f;
    [Tooltip("Robot start pose. If empty, the scene-start pose is used")]
    [SerializeField] private Transform spawnPoint;

    [Header("Episode")]
    [Tooltip("Hard cap on decision steps per episode. Guarantees the episode always resets.")]
    [SerializeField] private int maxEpisodeSteps = 4000;
    [Tooltip("MISSION MODE (Practice 7): for inference demos with the state machine. Disables all teleports/EndEpisode - the FSM owns the mission cycle. MUST be OFF for training!")]
    [SerializeField] private bool missionMode = false;

    [Header("Rewards (all fixed amounts - no multipliers/percentages anywhere)")]
    [Tooltip("Flat penalty per step while standing still (|gas| ~ 0). Moving forward alone gives nothing (0).")]
    [SerializeField] private float standingStillPenalty = 0.001f;
    [Tooltip("Flat reward per step for closing distance on a VISIBLE ball")]
    [SerializeField] private float towardBallReward = 0.003f;
    [Tooltip("Scale for centeringRewardScale * (1-|ServoAngle|) * (1-|BallAngle|), only while the ball is visible")]
    [SerializeField] private float centeringRewardScale = 0.001f;
    [Tooltip("Flat penalty per step for driving backwards while the ball IS visible")]
    [SerializeField] private float backwardVisiblePenalty = 0.001f;
    [Tooltip("Flat penalty per step for driving backwards while the ball is NOT visible")]
    [SerializeField] private float backwardBlindPenalty = 0.0001f;
    [Tooltip("Flat penalty for reversing gas/steer/camera direction (not just a large step - see suddenMoveSignificance)")]
    [SerializeField] private float suddenMovePenalty = 0.001f;
    [Tooltip("Minimum |value| for gas/steer/camCmd to count as a deliberate direction. Kept low so the SMOOTHED steer/gas (Input.GetAxis, or a smooth policy) still registers - a high threshold widens the near-zero dead band the ramp must cross, which the oscillation window then can't span")]
    [SerializeField] private float suddenMoveSignificance = 0.2f;
    [Tooltip("A direction reversal only counts as 'sudden' if the opposite significant reading appears within this many decisions of the last one. Must exceed how long a smoothed steer/gas takes to ramp across the dead band (else motor reversals never register), yet stay well under the straight-driving gap between two genuinely separate turns")]
    [SerializeField] private int oscillationWindowDecisions = 12;
    [Tooltip("Flat penalty when left/right IR is active (wall close on that side)")]
    [SerializeField] private float sideIRCriticalPenalty = 50f;
    [Tooltip("Flat penalty when the front ultrasonic reads under ultrasonicCriticalThreshold")]
    [SerializeField] private float ultrasonicCriticalPenalty = 50f;
    [SerializeField] private float ultrasonicCriticalThreshold = 0.2f;
    [SerializeField] private float fallPenalty = 3.0f;

    [Header("Claw hold / success (based on the raw GripperIR sensor - matches real hardware, not a physics-only 'IsHolding' state)")]
    [Tooltip("Decisions per reward tick. 5 decisions = 0.5 s at Decision Period 5 / 50 Hz physics")]
    [SerializeField] private int gripRewardIntervalDecisions = 5;
    [Tooltip("Decisions to hold before success (episode ends). 20 decisions = ~2 s at Decision Period 5 / 50 Hz physics (matches the real gripper closing time)")]
    [SerializeField] private int gripHoldMaxDecisions = 20;
    [Tooltip("Flat reward granted every gripRewardIntervalDecisions while the claw IR sensor stays continuously active")]
    [SerializeField] private float gripHoldTickBonus = 50f;
    [Tooltip("Flat penalty when the claw IR sensor was active and then deactivates before reaching gripHoldMaxDecisions (ball touched the claw and was lost)")]
    [SerializeField] private float gripLostPenalty = 50f;

    [Header("Debug - live reward breakdown (read-only, updates every step in Play mode)")]
    [SerializeField] private float dbg_StandingStillApplied;
    [SerializeField] private float dbg_TowardBallApplied;
    [SerializeField] private float dbg_CenteringApplied;
    [SerializeField] private float dbg_BackwardApplied;
    [SerializeField] private float dbg_SideIRCriticalApplied;
    [SerializeField] private float dbg_UltrasonicCriticalApplied;
    [SerializeField] private float dbg_SuddenMoveApplied;
    [SerializeField] private float dbg_GripHoldBonusApplied;
    [SerializeField] private float dbg_GripLostApplied;
    [SerializeField] private float dbg_FallPenaltyApplied;

    [Header("Observation normalization")]
    [SerializeField] private float timeSinceBallCap = 10f;  // s

    // ---- Rigidbody / start poses ----
    private Rigidbody rb;
    private Vector3 startPos;
    private Quaternion startRot;
    private Quaternion cameraServoBase;
    private float ballStartY;

    // ---- internal state ----
    private float servoAngle;
    private float lastKnownBallAngle;
    private float timeSinceBall;
    private float prevWorldDistance;
    // Last SIGNIFICANT (|value| > suddenMoveSignificance) direction seen for each channel
    // (-1, +1, or 0 = none recorded yet), and how many decisions ago that was. A reversal
    // only counts as "sudden" if the opposite significant direction reappears within
    // oscillationWindowDecisions - otherwise it's a normal, well-separated turn (e.g. went
    // right a while ago, now going left to navigate), not left-right-left-right jitter.
    private float lastSigGas, lastSigSteer, lastSigCam;
    private int gasSigAge = 999999, steerSigAge = 999999, camSigAge = 999999;
    private int episodeSteps;
    private int gripHoldDecisions;
    private int gripRewardTicksGranted;

    /// <summary>True when connected to the Python trainer (used later for domain randomization / ROSBridge gating).</summary>
    public bool IsTraining => Academy.Instance.IsCommunicatorOn;

    // ============ PUBLIC API FOR BrainDebugHUD ============
    public float Obs01_Ultrasonic        { get; private set; }
    public int   Obs02_LeftIR            { get; private set; }
    public int   Obs03_RightIR           { get; private set; }
    public int   Obs04_GripperIR         { get; private set; }
    public float Obs05_BallAngle         { get; private set; }
    public float Obs06_BallDistance      { get; private set; }
    public float Obs07_LastKnownAngle    { get; private set; }
    public float Obs08_BallVisible       { get; private set; }
    public float Obs09_ServoAngleNorm    { get; private set; }
    public float Obs10_HasBall           { get; private set; }
    public float Obs11_TimeSinceBallNorm { get; private set; }

    public float ActGas       { get; private set; }
    public float ActSteer     { get; private set; }
    public float ActCameraCmd { get; private set; }

    public float StepReward       { get; private set; }
    public float CumulativeReward => GetCumulativeReward();
    // =====================================================

    private Vector3 ArenaCenter => arenaCenter != null ? arenaCenter.position : Vector3.zero;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        startPos = spawnPoint != null ? spawnPoint.position : transform.position;
        startRot = spawnPoint != null ? spawnPoint.rotation : transform.rotation;

        if (cameraServo != null) cameraServoBase = cameraServo.localRotation;

        if (ball == null && !string.IsNullOrEmpty(ballTag))
        {
            var go = GameObject.FindWithTag(ballTag);
            if (go != null) ball = go.transform;
        }
        if (ball != null) ballStartY = ball.position.y;
    }

    public override void OnEpisodeBegin()
    {
        episodeSteps = 0;

        // MISSION MODE: the state machine owns the world - no teleports,
        // no ball respawn, no obstacle shuffle. Only internal counters reset.
        if (missionMode)
        {
            lastKnownBallAngle = 0f;
            timeSinceBall = timeSinceBallCap;
            lastSigGas = lastSigSteer = lastSigCam = 0f;
            gasSigAge = steerSigAge = camSigAge = 999999;
            gripHoldDecisions = 0;
            gripRewardTicksGranted = 0;
            prevWorldDistance = FlatDistanceToBall();
            return;
        }

        // Competition rule: scatter the obstacle belt BEFORE spawning the ball,
        // so the ball's overlap check sees the new obstacle positions
        if (obstacleRandomizer != null) obstacleRandomizer.Shuffle();

        // Reset robot: back to spawn, with sideways jitter along its start side.
        // Teleport through the rigidbody so PhysX state matches the new pose.
        rb.linearVelocity = Vector3.zero;   // Unity < 6: rb.velocity
        rb.angularVelocity = Vector3.zero;
        Vector3 spawn = startPos;
        if (robotSpawnJitter > 0f)
            spawn += (startRot * Vector3.right) * Random.Range(-robotSpawnJitter, robotSpawnJitter);
        rb.position = spawn;
        rb.rotation = startRot;
        transform.SetPositionAndRotation(spawn, startRot);

        // Reset camera servo
        servoAngle = 0f;
        if (cameraServo != null) cameraServo.localRotation = cameraServoBase;

        // Ball spawns in the FAR band AHEAD of the robot's start direction
        // (competition layout: robot starts on one side facing in, ball lands
        // randomly in the opposite half, behind the obstacle belt)
        if (ball != null)
        {
            Vector3 fwd  = startRot * Vector3.forward;
            Vector3 side = startRot * Vector3.right;
            Vector3 c = ArenaCenter;
            float lim = arenaHalf - 0.25f;   // keep the ball inside the walls
            float ballR = ball.localScale.x * 0.5f + 0.02f;
            Vector3 p = ball.position; int guard = 0;
            bool ok = false;
            while (!ok && guard < 40)
            {
                guard++;
                p = startPos
                  + fwd  * Random.Range(ballMinForward, ballMaxForward)
                  + side * Random.Range(-ballHalfWidth, ballHalfWidth);
                p.y = ballStartY;

                // clamp inside the arena bounds
                p.x = Mathf.Clamp(p.x, c.x - lim, c.x + lim);
                p.z = Mathf.Clamp(p.z, c.z - lim, c.z + lim);

                // not inside anything solid: allow only the floor (arenaCenter)
                // and the ball itself
                ok = true;
                foreach (var col in Physics.OverlapSphere(p, ballR))
                {
                    if (ball != null && col.transform == ball) continue;              // the ball itself
                    if (arenaCenter != null &&
                        (col.transform == arenaCenter || col.transform.IsChildOf(arenaCenter)))
                        continue;                                                     // the floor
                    ok = false; break;                                                // wall / obstacle / robot
                }
            }

            ball.position = p;
            var brb = ball.GetComponent<Rigidbody>();
            if (brb != null) { brb.isKinematic = false; brb.linearVelocity = Vector3.zero; brb.angularVelocity = Vector3.zero; }
            var bcol = ball.GetComponent<Collider>();
            if (bcol != null) bcol.enabled = true;
        }

        // Reset internal state
        lastKnownBallAngle = 0f;
        timeSinceBall = timeSinceBallCap;
        lastSigGas = lastSigSteer = lastSigCam = 0f;
        gasSigAge = steerSigAge = camSigAge = 999999;
        gripHoldDecisions = 0;
        gripRewardTicksGranted = 0;
        prevWorldDistance = FlatDistanceToBall();

        // Domain Randomization (Practice 5): physics + latency queue reset
        if (randomizer != null) randomizer.ApplyEpisodeRandomization(IsTraining);
    }

    private void FixedUpdate()
    {
        // Grows continuously; reset in CollectObservations when the ball is visible
        timeSinceBall += Time.fixedDeltaTime;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. Ultrasonic: take the nearer side (0 = touching .. 1 = clear)
        Obs01_Ultrasonic = sensors != null
            ? Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight)
            : 1f;
        if (randomizer != null)
            Obs01_Ultrasonic = randomizer.NoisySonar(Obs01_Ultrasonic, IsTraining);

        // 2-4. IR
        Obs02_LeftIR    = sensors != null ? sensors.LeftIR : 0;
        Obs03_RightIR   = sensors != null ? sensors.RightIR : 0;
        Obs04_GripperIR = sensors != null ? sensors.GripperIR : 0;

        // 5-8. Camera / YOLO. Source: real robot (Practice 7 HIL) when RealVision
        // is assigned and switched on, otherwise the simulated camera.
        bool useReal = realVision != null && realVision.useYOLO;
        bool visible = useReal ? realVision.IsVisible
                               : (yolo != null && yolo.IsVisible);
        float visAngle = useReal ? realVision.RelativeAngle
                                 : (yolo != null ? yolo.RelativeAngle : 0f);
        float visDist  = useReal ? realVision.NormalizedDistance
                                 : (yolo != null ? yolo.NormalizedDistance : 1f);
        if (randomizer != null && rb != null)
            visible = randomizer.FilterBallVisibility(visible, rb.angularVelocity.magnitude, IsTraining);
        Obs08_BallVisible  = visible ? 1f : 0f;
        Obs05_BallAngle    = visible ? visAngle : 0f;
        Obs06_BallDistance = visible ? visDist : 1f;
        if (visible) { lastKnownBallAngle = visAngle; timeSinceBall = 0f; }
        Obs07_LastKnownAngle = lastKnownBallAngle;

        // 9. Camera servo
        Obs09_ServoAngleNorm = Mathf.Clamp(servoAngle / maxServoAngle, -1f, 1f);

        // 10. Has ball: mirrors the raw GripperIR sensor directly (not the physics-only
        // "IsHolding" state) - the real robot only ever has the raw sensor to go on.
        Obs10_HasBall = Obs04_GripperIR == 1 ? 1f : 0f;

        // 11. Time since last detection
        Obs11_TimeSinceBallNorm = Mathf.Clamp01(timeSinceBall / timeSinceBallCap);

        // --- Feed strictly in the Practice 3 table order ---
        sensor.AddObservation(Obs01_Ultrasonic);
        sensor.AddObservation(Obs02_LeftIR);
        sensor.AddObservation(Obs03_RightIR);
        sensor.AddObservation(Obs04_GripperIR);
        sensor.AddObservation(Obs05_BallAngle);
        sensor.AddObservation(Obs06_BallDistance);
        sensor.AddObservation(Obs07_LastKnownAngle);
        sensor.AddObservation(Obs08_BallVisible);
        sensor.AddObservation(Obs09_ServoAngleNorm);
        sensor.AddObservation(Obs10_HasBall);
        sensor.AddObservation(Obs11_TimeSinceBallNorm);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        StepReward = 0f;
        episodeSteps++;

        float gas    = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steer  = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float camCmd = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        ActGas = gas; ActSteer = steer; ActCameraCmd = camCmd;

        // Domain Randomization: action latency FIFO (no-op when latency is 0)
        if (randomizer != null) randomizer.DelayActions(ref gas, ref steer, ref camCmd);

        // Drive (SetCommand clamps to [-1,1] itself)
        if (track != null) track.SetCommand(gas, steer);

        // Camera pan: rotate around the WORLD vertical axis (robust to any
        // tilted parent hierarchy in the imported model - guarantees the pan
        // is left/right, never up/down)
        servoAngle = Mathf.Clamp(servoAngle + camCmd * servoSpeed * Time.fixedDeltaTime,
                                 -maxServoAngle, maxServoAngle);
        if (cameraServo != null)
            cameraServo.localRotation = cameraServoBase *
                Quaternion.AngleAxis(servoAngle,
                    cameraServo.parent != null
                        ? cameraServo.parent.InverseTransformDirection(Vector3.up)
                        : Vector3.up);

        ComputeRewards(gas, steer, camCmd);
    }

    private void ComputeRewards(float gas, float steer, float camCmd)
    {
        dbg_GripHoldBonusApplied = 0f;
        dbg_GripLostApplied = 0f;

        // ---- Claw hold tracking: purely observational - driving is NEVER frozen while
        // the sensor is active. Flat, fixed bonus per tick (NOT a % of cumulative reward -
        // avoids exponential growth and removes any incentive to stall/inflate a baseline
        // before grabbing, since the bonus no longer depends on prior accumulated reward).
        bool gripActive = Obs04_GripperIR == 1;
        if (gripActive)
        {
            if (!missionMode)
            {
                gripHoldDecisions++;
                int ticksReached = gripHoldDecisions / gripRewardIntervalDecisions;
                while (gripRewardTicksGranted < ticksReached)
                {
                    gripRewardTicksGranted++;
                    Add(gripHoldTickBonus);
                    dbg_GripHoldBonusApplied += gripHoldTickBonus;
                }

                if (gripHoldDecisions >= gripHoldMaxDecisions)
                {
                    EndEpisode();   // success
                    return;
                }
            }
        }
        else
        {
            // Sensor was active and just dropped before reaching success - the ball
            // touched the claw and was lost before the hold completed. Flat penalty,
            // same fixed scale as the hold-tick bonus (not tied to cumulative reward).
            if (gripHoldDecisions > 0 && !missionMode)
            {
                Add(-gripLostPenalty);
                dbg_GripLostApplied = -gripLostPenalty;
            }
            gripHoldDecisions = 0;
            gripRewardTicksGranted = 0;
        }

        // Real, occlusion-aware visibility - Obs08_BallVisible already went through
        // SimulatedYoloCamera's raycast line-of-sight check (or RealVision on hardware),
        // so this means "actually seen", not just "somewhere in the FOV cone through a wall".
        bool ballVisible = Obs08_BallVisible > 0.5f;
        float dist = ball != null ? FlatDistanceToBall() : float.MaxValue;

        // Ground-truth world distance must never leak into reward while the robot can't
        // actually see the ball - otherwise driving blind gets rewarded/punished for a
        // distance change it has no way of perceiving. Only compared/updated on steps
        // where the ball is actually visible.
        bool movingToward = false;
        if (ball != null && ballVisible)
        {
            movingToward = dist < prevWorldDistance;
            prevWorldDistance = dist;
        }

        // --- Standing still: flat penalty. Moving forward alone gives nothing (0). ---
        bool standingStill = Mathf.Abs(gas) <= 0.0001f;
        if (standingStill)
        {
            Add(-standingStillPenalty);
            dbg_StandingStillApplied = -standingStillPenalty;
        }
        else
        {
            dbg_StandingStillApplied = 0f;
        }

        // --- Backward: flat penalty, graded by whether the ball is visible right now ---
        if (gas < 0f)
        {
            float penalty = ballVisible ? backwardVisiblePenalty : backwardBlindPenalty;
            Add(-penalty);
            dbg_BackwardApplied = -penalty;
        }
        else
        {
            dbg_BackwardApplied = 0f;
        }

        // --- Toward ball: flat reward, only counted while the ball is actually visible ---
        if (movingToward)
        {
            Add(towardBallReward);
            dbg_TowardBallApplied = towardBallReward;
        }
        else
        {
            dbg_TowardBallApplied = 0f;
        }

        // --- Centering: flat-scaled formula, only while visible ---
        if (ballVisible)
        {
            float centering = centeringRewardScale * (1f - Mathf.Abs(Obs09_ServoAngleNorm)) * (1f - Mathf.Abs(Obs05_BallAngle));
            Add(centering);
            dbg_CenteringApplied = centering;
        }
        else
        {
            dbg_CenteringApplied = 0f;
        }

        // --- Critical wall proximity: flat penalties, independent of movement and of each
        // other (both triggering the same step just adds both penalties, no compounding).
        if (Obs02_LeftIR == 1 || Obs03_RightIR == 1)
        {
            Add(-sideIRCriticalPenalty);
            dbg_SideIRCriticalApplied = -sideIRCriticalPenalty;
        }
        else
        {
            dbg_SideIRCriticalApplied = 0f;
        }

        if (Obs01_Ultrasonic < ultrasonicCriticalThreshold)
        {
            Add(-ultrasonicCriticalPenalty);
            dbg_UltrasonicCriticalApplied = -ultrasonicCriticalPenalty;
        }
        else
        {
            dbg_UltrasonicCriticalApplied = 0f;
        }

        // --- Sudden move penalty: covers motors (gas/steer) AND the camera servo command.
        // Compares against the last SIGNIFICANT direction per channel (|value| >
        // suddenMoveSignificance), not the immediately preceding step - a raw step-to-step
        // comparison misses oscillation that ramps gradually through zero (e.g. steer/gas
        // driven by Input.GetAxis smoothing in Heuristic, or a smooth policy output), since
        // every individual small step during the ramp has a tiny delta.
        //
        // A reversal only counts as "sudden" if the opposite significant direction shows up
        // within oscillationWindowDecisions of the last one (*Age tracks decisions since the
        // channel was last significant, of EITHER sign). Otherwise a normal turn taken long
        // after the previous one (e.g. went right a while ago, now going left to navigate)
        // would get flagged just for being opposite - it isn't oscillation, just driving.
        gasSigAge++; steerSigAge++; camSigAge++;
        bool suddenMove = false;

        if (Mathf.Abs(gas) > suddenMoveSignificance)
        {
            float sign = Mathf.Sign(gas);
            if (lastSigGas != 0f && sign != lastSigGas && gasSigAge <= oscillationWindowDecisions) suddenMove = true;
            lastSigGas = sign;
            gasSigAge = 0;
        }
        if (Mathf.Abs(steer) > suddenMoveSignificance)
        {
            float sign = Mathf.Sign(steer);
            if (lastSigSteer != 0f && sign != lastSigSteer && steerSigAge <= oscillationWindowDecisions) suddenMove = true;
            lastSigSteer = sign;
            steerSigAge = 0;
        }
        if (Mathf.Abs(camCmd) > suddenMoveSignificance)
        {
            float sign = Mathf.Sign(camCmd);
            if (lastSigCam != 0f && sign != lastSigCam && camSigAge <= oscillationWindowDecisions) suddenMove = true;
            lastSigCam = sign;
            camSigAge = 0;
        }

        if (suddenMove)
        {
            Add(-suddenMovePenalty);
            dbg_SuddenMoveApplied = -suddenMovePenalty;
        }
        else
        {
            dbg_SuddenMoveApplied = 0f;
        }

        // --- Terminal conditions ---
        // (Success is handled by the claw hold tracking at the top of this method.)

        dbg_FallPenaltyApplied = 0f;

        // Mission mode: the FSM owns the mission - never terminate episodes.
        if (missionMode) return;

        // Failure: fell through the floor or left the arena.
        Vector3 c = ArenaCenter;
        bool outside = Mathf.Abs(transform.position.x - c.x) > arenaHalf + 0.3f
                    || Mathf.Abs(transform.position.z - c.z) > arenaHalf + 0.3f
                    || transform.position.y < c.y - fallDrop
                    || transform.position.y > c.y + 1.5f;   // launched into the air
        if (outside)
        {
            Add(-fallPenalty);
            dbg_FallPenaltyApplied = -fallPenalty;
            EndEpisode();
            return;
        }

        // Time up: truncate without a big penalty so the episode always resets.
        if (episodeSteps >= maxEpisodeSteps)
            EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        // GetAxis (smoothed), not GetAxisRaw: raw instant -1/0/+1 made manual keyboard
        // driving violently bang-bang (visibly shaking left-right from the robot's own
        // camera). This only affects Heuristic/manual testing - during actual training the
        // policy's continuous outputs never go through Input Manager at all, so the sudden-
        // move detection still works correctly for real training regardless of this.
        cont[0] = Input.GetAxis("Vertical");     // W/S -> gas
        cont[1] = Input.GetAxis("Horizontal");   // A/D -> steer
        cont[2] = (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f); // Q/E -> camera
    }

    // ---- helpers ----
    private float FlatDistanceToBall()
    {
        if (ball == null) return 0f;
        Vector3 a = transform.position; a.y = 0f;
        Vector3 b = ball.position;      b.y = 0f;
        return Vector3.Distance(a, b);
    }

    private void Add(float r) { AddReward(r); StepReward += r; }
}
