using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

/// <summary>
/// Domain Randomization module (Practice 5), kept in its own component so the
/// team can tune it independently of RobotBrain.
///
/// Covers all four camp randomization steps:
///   1. Physical: robot mass, motor speeds, ball mass/scale (per episode)
///   2. Sonar white noise (per observation)
///   3. YOLO burst dropout on fast rotation (per observation)
///   4. Action latency FIFO queue (per action)
///
/// Usage: add to the robot object, drag references, then RobotBrain calls the
/// hooks. Every feature has its own toggle + a master switch. If the component
/// is missing or disabled, RobotBrain behaves exactly as before.
/// NOTE: randomization only activates when isTraining == true (trainer
/// connected), so Heuristic driving and real-robot inference stay clean.
/// </summary>
public class DomainRandomizer : MonoBehaviour
{
    [Header("Master switch")]
    [Tooltip("Turn ALL randomization on/off with one checkbox")]
    public bool enableRandomization = true;

    [Header("References")]
    [SerializeField] private Rigidbody robotRb;
    [SerializeField] private TrackController track;
    [SerializeField] private Transform ball;

    [Header("1. Robot physics (per episode)")]
    public bool randomizeRobotMass = true;
    public Vector2 robotMassRange = new Vector2(1.0f, 4.0f);   // base 2.5 kg
    public bool randomizeMotors = true;
    public Vector2 moveSpeedRange = new Vector2(0.3f, 0.7f);   // base 0.57 m/s
    public Vector2 turnSpeedRange = new Vector2(80f, 160f);    // base 120 deg/s

    [Header("1b. Ball physics (per episode)")]
    public bool randomizeBall = true;
    [Tooltip("Ball mass multiplier range relative to its original mass")]
    public Vector2 ballMassMul = new Vector2(0.5f, 2.0f);      // +-100%
    [Tooltip("Ball scale multiplier range relative to its original scale")]
    public Vector2 ballScaleMul = new Vector2(0.8f, 1.2f);     // +-20%

    [Header("2. Sonar noise (per observation)")]
    public bool sonarNoise = true;
    [Tooltip("White noise amplitude added to the normalized sonar reading")]
    public float sonarNoiseAmp = 0.05f;                        // +-5%

    [Header("3. YOLO burst dropout (per observation)")]
    public bool yoloDropout = true;
    [Tooltip("Angular speed (rad/s) above which dropout can trigger")]
    public float dropoutAngularSpeed = 0.5f;
    [Tooltip("Chance per step to start a dropout burst while rotating fast")]
    [Range(0f, 1f)] public float dropoutChance = 0.15f;
    [Tooltip("Burst length range in steps (min inclusive, max exclusive)")]
    public Vector2Int dropoutSteps = new Vector2Int(5, 16);

    [Header("4. Action latency queue (per action)")]
    public bool actionLatency = true;
    [Tooltip("Latency range in decision steps (min inclusive, max exclusive). 8-14 steps = 160-260 ms")]
    public Vector2Int latencySteps = new Vector2Int(8, 14);

    // ---- internal state ----
    private float ballBaseMass = -1f;
    private Vector3 ballBaseScale;
    private int burstDropoutRemaining;
    private readonly Queue<float[]> actionBuffer = new Queue<float[]>();
    private int currentLatency;

    private void Awake()
    {
        // Remember the ball's original mass/scale once, so per-episode
        // randomization multiplies the BASE values and never compounds.
        if (ball != null)
        {
            ballBaseScale = ball.localScale;
            var brb = ball.GetComponent<Rigidbody>();
            if (brb != null) ballBaseMass = brb.mass;
        }
    }

    // ================= Hook 1: call from OnEpisodeBegin =================
    public void ApplyEpisodeRandomization(bool isTraining)
    {
        // --- LECTURA DE RUIDO DESDE EL YAML ---
        // 0 = Off (Fases 0 a 5), 1 = On (Fase 6 Sim2Real)
        float noiseParam = Academy.Instance.EnvironmentParameters.GetWithDefault("sim2real_noise", 0f);
        
        // El master switch o las opciones de vision/sonar se adaptan al parametro
        bool allowNoise = noiseParam > 0.5f;
        
        // Ajustar dinámicamente los toggles de visión/sensor para la fase actual
        sonarNoise = allowNoise;
        yoloDropout = allowNoise;

        InitLatencyQueue(isTraining);
        burstDropoutRemaining = 0;

        if (!Active(isTraining)) return;

        if (randomizeRobotMass && robotRb != null)
            robotRb.mass = Random.Range(robotMassRange.x, robotMassRange.y);

        if (randomizeMotors && track != null)
            track.SetMotorParams(
                Random.Range(moveSpeedRange.x, moveSpeedRange.y),
                Random.Range(turnSpeedRange.x, turnSpeedRange.y));

        if (randomizeBall && ball != null)
        {
            ball.localScale = ballBaseScale * Random.Range(ballScaleMul.x, ballScaleMul.y);
            var brb = ball.GetComponent<Rigidbody>();
            if (brb != null && ballBaseMass > 0f)
                brb.mass = ballBaseMass * Random.Range(ballMassMul.x, ballMassMul.y);
        }
    }

    // ================= Hook 2: call from CollectObservations =================
    /// <summary>Adds white noise to the normalized sonar reading.</summary>
    public float NoisySonar(float value, bool isTraining)
    {
        if (!Active(isTraining) || !sonarNoise) return value;
        return Mathf.Clamp01(value + Random.Range(-sonarNoiseAmp, sonarNoiseAmp));
    }

    /// <summary>
    /// Burst dropout: while the robot spins fast, the (simulated) camera can
    /// lose the ball for several consecutive steps. Call once per observation.
    /// </summary>
    public bool FilterBallVisibility(bool rawVisible, float angularSpeed, bool isTraining)
    {
        if (!Active(isTraining) || !yoloDropout) return rawVisible;

        if (burstDropoutRemaining > 0)
        {
            burstDropoutRemaining--;
        }
        else if (angularSpeed > dropoutAngularSpeed && Random.value < dropoutChance)
        {
            burstDropoutRemaining = Random.Range(dropoutSteps.x, dropoutSteps.y);
        }

        return rawVisible && burstDropoutRemaining == 0;
    }

    // ================= Hook 3: call from OnActionReceived =================
    /// <summary>
    /// FIFO latency: enqueue the fresh continuous actions, dequeue the stale
    /// ones. With latency 0 (inference/heuristic) it is a pass-through.
    /// </summary>
    public void DelayActions(ref float gas, ref float steer, ref float camCmd)
    {
        if (currentLatency <= 0) return;

        actionBuffer.Enqueue(new float[] { gas, steer, camCmd });
        float[] delayed = actionBuffer.Dequeue();
        gas = delayed[0];
        steer = delayed[1];
        camCmd = delayed[2];
    }

    // ---- helpers ----
    private bool Active(bool isTraining) => enableRandomization && isTraining;

    private void InitLatencyQueue(bool isTraining)
    {
        currentLatency = (Active(isTraining) && actionLatency)
            ? Random.Range(latencySteps.x, latencySteps.y)
            : 0;

        actionBuffer.Clear();
        for (int i = 0; i < currentLatency; i++)
            actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
    }
}
