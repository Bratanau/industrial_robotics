using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// Interceptor GFS-X AI agent. Glues the team scripts (TrackController,
/// VirtualSensors, SimulatedYoloCamera, GripperController) into one "brain":
///   - collects 15 observations (order strictly matches the Practice 3 table),
///   - dispatches 3 continuous + 1 discrete action,
///   - computes bounded rewards,
///   - exposes the public fields that BrainDebugHUD reads.
/// Inherits Agent (ML-Agents). Attach to the "robot" object.
///
/// Behavior Parameters on the robot MUST match:
///   Behavior Name = GFSX_Brain (same as config.yaml)
///   Vector Observation -> Space Size = 15, Stacked Vectors = 4
///   Continuous Actions = 3, Discrete Branches = 1, Branch 0 Size = 3
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
    [SerializeField] private GripperController gripper;

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

    [Header("Rewards")]
    [SerializeField] private float approachReward = 2.0f;   // per meter of approach
    [SerializeField] private float approachDeltaClamp = 0.15f; // max |dist change| counted per step
    [SerializeField] private float nearBoost = 2.0f;        // multiplier when close to ball
    [SerializeField] private float centerReward = 0.003f;   // for centering camera/body on ball
    [SerializeField] private float actionRatePenalty = 0.005f;
    [Tooltip("Penalty each time the gripper COMMAND changes (0/1/2). Stops the policy from chattering the claw servo, which wears the real hardware")]
    [SerializeField] private float gripChatterPenalty = 0.01f;
    [SerializeField] private float wallPenalty = 0.02f;
    [SerializeField] private float stepPenalty = 0.0005f;
    [SerializeField] private float reversePenalty = 0.004f; // penalty per step for driving backwards
    [Tooltip("Distance (m) under which the robot is rewarded for slowing down before the grab")]
    [SerializeField] private float slowdownRadius = 0.4f;
    [SerializeField] private float slowdownReward = 0.004f; // reward for being slow when near the ball
    [SerializeField] private float grabReward = 5.0f;
    [Tooltip("How many agent decisions the robot must HOLD the ball, standing still, before the episode ends with success. 20 decisions = ~2 s at Decision Period 5 / 50 Hz physics (matches the real gripper closing time)")]
    [SerializeField] private int holdDecisions = 20;
    [Tooltip("Small reward per decision while holding (keeps the value signal alive during the forced stop)")]
    [SerializeField] private float holdReward = 0.02f;
    [SerializeField] private float fallPenalty = 3.0f;
    [Tooltip("Penalty each time the hull/bumper physically hits the ball (teaches gentle approach with the gripper, not ramming)")]
    [SerializeField] private float ballBumpPenalty = 0.05f;

    [Header("Observation normalization")]
    [SerializeField] private float maxSpeed = 0.8f;         // m/s of the real GFS-X
    [SerializeField] private float timeSinceBallCap = 10f;  // s
    [Tooltip("Real GFS-X has no wheel encoders, so dX/dZ odometry (obs 11-12) does not exist on hardware. ON = feed zeros instead (train the policy to live without odometry before Sim2Real)")]
    [SerializeField] private bool zeroOdometry = false;

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
    private float prevGas, prevSteer;
    private int prevGrip;
    private int episodeSteps;
    private int holdCounter;

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
    public float Obs11_DxNorm            { get; private set; }
    public float Obs12_DzNorm            { get; private set; }
    public float Obs13_HeadingNorm       { get; private set; }
    public float Obs14_Speed             { get; private set; }
    public float Obs15_TimeSinceBallNorm { get; private set; }

    public float ActGas       { get; private set; }
    public float ActSteer     { get; private set; }
    public float ActCameraCmd { get; private set; }
    public int   ActGripCmd   { get; private set; }

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
            prevGas = prevSteer = 0f;
            holdCounter = 0;
            prevWorldDistance = FlatDistanceToBall();
            return;
        }

        // If we were holding something, release it (restores ball physics and parent)
        if (gripper != null && gripper.IsHolding) gripper.ReleaseCommand();

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
        prevGas = prevSteer = 0f;
        prevGrip = 0;
        holdCounter = 0;
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
        if (visible) { lastKnownBallAngle = yolo.RelativeAngle; timeSinceBall = 0f; }
        Obs07_LastKnownAngle = lastKnownBallAngle;

        // 9. Camera servo
        Obs09_ServoAngleNorm = Mathf.Clamp(servoAngle / maxServoAngle, -1f, 1f);

        // 10. Are we holding the ball
        Obs10_HasBall = (gripper != null && gripper.IsHolding) ? 1f : 0f;

        // 11-12. Offset from start (odometry). Real GFS-X has no encoders,
        // so with zeroOdometry ON the policy learns to work without it.
        if (zeroOdometry)
        {
            Obs11_DxNorm = 0f;
            Obs12_DzNorm = 0f;
        }
        else
        {
            Vector3 d = transform.position - startPos;
            Obs11_DxNorm = Mathf.Clamp(d.x / arenaHalf, -1f, 1f);
            Obs12_DzNorm = Mathf.Clamp(d.z / arenaHalf, -1f, 1f);
        }

        // 13. Robot heading (-1..1)
        float heading = transform.eulerAngles.y;
        if (heading > 180f) heading -= 360f;
        Obs13_HeadingNorm = heading / 180f;

        // 14. Speed
        Obs14_Speed = rb != null ? Mathf.Clamp01(rb.linearVelocity.magnitude / maxSpeed) : 0f;

        // 15. Time since last detection
        Obs15_TimeSinceBallNorm = Mathf.Clamp01(timeSinceBall / timeSinceBallCap);

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
        sensor.AddObservation(Obs11_DxNorm);
        sensor.AddObservation(Obs12_DzNorm);
        sensor.AddObservation(Obs13_HeadingNorm);
        sensor.AddObservation(Obs14_Speed);
        sensor.AddObservation(Obs15_TimeSinceBallNorm);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        StepReward = 0f;
        episodeSteps++;

        float gas    = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steer  = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float camCmd = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        int   grip   = actions.DiscreteActions[0];   // 0 idle / 1 close / 2 open

        ActGas = gas; ActSteer = steer; ActCameraCmd = camCmd; ActGripCmd = grip;

        // ---- HOLD PHASE: ball is gripped -> stand still and wait for the
        // (real-world) gripper to fully close before the success is counted.
        if (gripper != null && gripper.IsHolding)
        {
            StepReward = 0f;
            if (track != null) track.SetCommand(0f, 0f);   // forced stop

            // Mission mode: just hold position; the state machine will disable
            // this agent and drive the delivery leg itself.
            if (missionMode) return;

            holdCounter++;
            Add(holdReward);

            if (holdCounter >= holdDecisions)
            {
                Add(grabReward);
                EndEpisode();
            }
            return;                                        // ignore net actions while holding
        }
        holdCounter = 0;                                   // lost the ball -> restart the hold

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

        // Gripper
        if (gripper != null)
        {
            if (grip == 1) gripper.GripCommand();
            else if (grip == 2) gripper.ReleaseCommand();
        }

        ComputeRewards(gas, steer);

        prevGas = gas; prevSteer = steer;
    }

    private void ComputeRewards(float gas, float steer)
    {
        float dist = ball != null ? FlatDistanceToBall() : float.MaxValue;

        // 1. Distance Delta: reward for approaching, delta clamped so a ball
        //    nudge / teleport can never produce a huge single-step spike.
        if (ball != null)
        {
            float delta = Mathf.Clamp(prevWorldDistance - dist, -approachDeltaClamp, approachDeltaClamp);
            float boost = dist < 0.5f ? nearBoost : 1f;
            Add(delta * approachReward * boost);
            prevWorldDistance = dist;
        }

        // 2. Look at the ball (correct trajectory): reward for centering it in view.
        //    Uses the same source as observations (sim camera or RealVision).
        if (Obs08_BallVisible > 0.5f)
            Add(centerReward * (1f - Mathf.Abs(Obs05_BallAngle)));

        // 3. Slow down near the ball: reward being slow when close, so it doesn't
        //    ram the ball at full speed right before the grab.
        if (ball != null && dist < slowdownRadius)
        {
            float speedNorm = rb != null ? Mathf.Clamp01(rb.linearVelocity.magnitude / maxSpeed) : 0f;
            Add(slowdownReward * (1f - speedNorm));
        }

        // 4. Action Rate Penalty: discourage jerky motor commands.
        Add(-actionRatePenalty * (Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer)));

        // 4b. Gripper chatter: penalize CHANGING the claw command (not holding it).
        if (ActGripCmd != prevGrip)
            Add(-gripChatterPenalty);
        prevGrip = ActGripCmd;

        // 5. Reverse penalty: small cost for driving backwards without need.
        if (gas < 0f)
            Add(reversePenalty * gas); // gas<0 => negative reward, scaled by how hard it reverses

        // 6. Wall Proximity: penalty for getting critically close to walls (US + IR).
        if (sensors != null)
        {
            if (sensors.LeftIR == 1 || sensors.RightIR == 1 || sensors.CenterIR == 1)
                Add(-wallPenalty);
            if (Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight) < 0.15f)
                Add(-wallPenalty);
        }

        // 7. Small per-step penalty so it doesn't stall.
        Add(-stepPenalty);

        // --- Terminal conditions ---
        // (Success is handled by the HOLD phase in OnActionReceived: the robot
        // must hold the ball for holdDecisions before the episode ends.)

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
        cont[0] = Input.GetAxis("Vertical");     // W/S -> gas
        cont[1] = Input.GetAxis("Horizontal");   // A/D -> steer
        cont[2] = (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f); // Q/E -> camera

        var disc = actionsOut.DiscreteActions;
        if      (Input.GetKey(KeyCode.Space))     disc[0] = 1;  // grip
        else if (Input.GetKey(KeyCode.LeftShift)) disc[0] = 2;  // release
        else                                      disc[0] = 0;
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

    /// <summary>
    /// Physical contact between the hull/bumper and the ball. A properly
    /// gripped ball has its collider disabled, so any collision here means
    /// the robot rammed the ball with its body instead of using the gripper.
    /// </summary>
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag(ballTag))
            Add(-ballBumpPenalty);
    }
}
