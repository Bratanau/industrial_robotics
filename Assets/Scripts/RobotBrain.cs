using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// RobotBrain v6.1 — GFS-X (Яндекс×УрФУ)
/// =====================================================================
/// База: экономика наград из deathlydh/Unity-ROS-Autonomous-Agent (v27),
/// проверенная на 50M шагов. Адаптировано под наши компоненты и миссию.
///
/// МИССИЯ: ехать ПРЯМО на мяч, подъехать МЕДЛЕННО, чтобы мяч оказался
/// МЕЖДУ губками клешни, и ОСТАНОВИТЬСЯ, НЕ толкая его.
///
/// v5.0 — ПРОПОРЦИОНАЛЬНАЯ награда вместо бинарной. Бинарная оплачивала
///   ДРОЖЬ на месте, из-за чего оптимальной становилась политика с mean≈0,
///   и при Deterministic Inference робот просто стоял.
///
/// v5.3 — ДВОЙНЫЕ ВОРОТА центровки: мяч по центру кадра И камера не
///   отвёрнута сервоприводом. Иначе робот "центровался" поворотом камеры
///   и ехал физически боком — отсюда круги вокруг мяча.
///
/// v6.0 — Штраф за СДВИГ мяча вместо штрафа за КАСАНИЕ + ступенчатая
///   награда (касание -> удержание -> успех), чтобы агент не боялся
///   доводить манёвр до конца.
///
/// v6.1 — КРИТИЧЕСКИЙ ФИКС ЗАВИСАНИЯ ЭПИЗОДОВ.
///   В v6.0 блок "защёлки" (latchOnArrival) стоял ВЫШЕ проверки лимита
///   эпизода и делал return. Когда робот касался мяча, а мяч выкатывался,
///   агент попадал в защёлку и НИКОГДА не доходил до EndEpisode().
///   Симптом в логе: "No episode was completed since last summary" —
///   в прошлом прогоне так утекло 4 миллиона шагов, а затем
///   "corrupted size vs. prev_size" из-за роста буферов.
///   Теперь: лимит эпизода проверяется ПЕРВЫМ, а защёлка работает
///   ТОЛЬКО в инференсе (для демо на железе), не при обучении.
///
/// ⚠️ ЕСЛИ В ЛОГЕ ПОЯВИЛОСЬ "No episode was completed" — НЕМЕДЛЕННО СТОП.
///
/// INSPECTOR: Space Size = 18, Stacked Vectors = 4,
///            Continuous Actions = 3, Discrete Branches = 0.
/// =====================================================================
[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Robot Component References")]
    [SerializeField] private TrackController track;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private SimulatedYoloCamera yolo;
    [Tooltip("Práctica 7 (HIL): Receptor opcional de YOLO real.")]
    [SerializeField] private RealVision realVision;

    [Header("Domain Randomization & Diagnostics")]
    [SerializeField] private DomainRandomizer randomizer;
    [SerializeField] private ObstacleRandomizer obstacleRandomizer;
    [SerializeField] private DiagnosticLogger diagLogger;
    [SerializeField] private ROSBridge rosBridge;

    [Header("Camera Servo Controls")]
    [SerializeField] private Transform cameraServo;
    [SerializeField] private float maxServoAngle = 90f;   // deg
    [SerializeField] private float servoSpeed = 90f;      // deg/s

    [Header("Target and Arena")]
    [SerializeField] private Transform ball;
    [SerializeField] private string ballTag = "TargetBall";
    [SerializeField] private Transform arenaCenter;

    [Header("Ball Spawn Zone")]
    [Tooltip("Мяч ДОЛЖЕН рождаться в поле зрения камеры (FOV ≈ ±30°) и НЕ за препятствиями. Иначе награды за сближение не будет вообще.")]
    [SerializeField] private float ballMinForward = 0.6f;
    [SerializeField] private float ballMaxForward = 1.6f;
    [SerializeField] private float ballHalfWidth = 0.5f;
    [SerializeField] private float robotSpawnJitter = 0.3f;
    [SerializeField] private Transform spawnPoint;
    [SerializeField] private Transform finishArea;

    [Header("Episode & Mission Mode")]
    [SerializeField] private int maxEpisodeSteps = 1000;
    [Tooltip("MISSION MODE: демо-инференс. Отключает телепорты и EndEpisode.")]
    [SerializeField] private bool missionMode = false;
    [Tooltip("Защёлка: доехал -> стоим до конца. РАБОТАЕТ ТОЛЬКО В ИНФЕРЕНСЕ. При обучении она замораживала эпизоды навсегда.")]
    [SerializeField] private bool latchOnArrival = true;

    // ============== ЭКОНОМИКА НАГРАД ==============
    [Header("R1 — Approach (главный сигнал, ПРОПОРЦИОНАЛЬНЫЙ)")]
    [Tooltip("Итог = delta × scale × (2 + 4×(1−dist)) × alignFactor")]
    [SerializeField] private float approachScale = 1.0f;
    [SerializeField] private float approachSpikeFilter = 0.5f;

    [Header("R2 — Успех: ступени касание -> удержание -> стоп")]
    [Tooltip("Небольшая награда в момент ПЕРВОГО касания сенсором клешни")]
    [SerializeField] private float touchBonus = 0.5f;
    [Tooltip("Награда за каждый шаг удержания мяча в клешне СТОЯ")]
    [SerializeField] private float holdStillBonus = 0.05f;
    [Tooltip("Сколько шагов простоять с мячом в клешне до полного успеха")]
    [SerializeField] private int holdStillSteps = 25;
    [Tooltip("Финальная награда за выполненную миссию")]
    [SerializeField] private float reachSuccessReward = 5.0f;
    [Tooltip("Окно (шаги) после последнего видения мяча, в котором ИК клешни считается настоящим. Только для реального робота.")]
    [SerializeField] private int ballSeenWindowSteps = 150;

    [Header("R3 — ЕХАТЬ ПРЯМО: двойные ворота центровки")]
    [Tooltip("Ниже этой дистанции награда за сближение ТРЕБУЕТ движения прямо")]
    [SerializeField] private float alignRequiredDist = 0.5f;
    [Tooltip("Допуск по мячу в кадре")]
    [SerializeField] private float alignTolerance = 0.35f;
    [Tooltip("Допуск по сервоприводу: камера отвёрнута = корпус НЕ смотрит на мяч")]
    [SerializeField] private float servoTolerance = 0.25f;
    [Tooltip("Награда за возврат камеры к центру при видимом мяче (доворот КОРПУСОМ)")]
    [SerializeField] private float servoCenteringScale = 0.5f;
    [SerializeField] private float slowDownBonus = 0.005f;   // dist<0.30, газ малый
    [SerializeField] private float tooFastPenalty = 0.02f;   // dist<0.25, газ большой
    [SerializeField] private float alignBonus = 0.005f;      // dist<0.40, едет прямо
    [SerializeField] private float blindCrawlBonus = 0.003f; // мяч в слепой зоне
    [Tooltip("Награда за движение ВПЕРЁД по вектору подсказки, пока мяч не виден")]
    [SerializeField] private float hintFollowScale = 0.01f;
    [Tooltip("Допуск по углу подсказки: при большей ошибке награда за следование = 0")]
    [SerializeField] private float hintFollowTolerance = 0.3f;

    [Header("R4 — Штрафы по ДАТЧИКАМ (существуют и на железе)")]
    [SerializeField] private float sonarPenalty = 0.03f;     // градиентный
    [SerializeField] private float sonarThreshold = 0.12f;
    [SerializeField] private float sideIrPenalty = 0.01f;    // бинарный

    [Header("R5 — Гладкость и задний ход")]
    [SerializeField] private float actionRatePenalty = 0.05f;
    [SerializeField] private float reversePenalty = 0.005f;

    [Header("R6 — Анти-снегоуборщик (штраф за СДВИГ, не за касание)")]
    [SerializeField] private float ballPushPenalty = 0.05f;
    [Tooltip("Ниже этой скорости мяча контакт считается лёгким касанием, а не толчком")]
    [SerializeField] private float ballPushSpeedThreshold = 0.05f;
    [Tooltip("Скорость мяча, при которой штраф достигает максимума")]
    [SerializeField] private float ballPushSpeedNorm = 0.30f;

    [Header("R7 — Ползание и таймауты")]
    [Tooltip("Газ ниже порога считается стоянием — закрывает дыру gas=0.05")]
    [SerializeField] private float standingStillPenalty = 0.001f;
    [SerializeField] private float standingStillGasThreshold = 0.15f;
    [SerializeField] private float episodeTimeoutPenalty = 0.05f;
    [SerializeField] private float stuckPenalty = 0.5f;

    [Header("Debug — Live Reward Breakdown (Read-Only)")]
    [SerializeField] private float dbg_Approach;
    [SerializeField] private float dbg_Centering;
    [SerializeField] private float dbg_AlignFactor;
    [SerializeField] private float dbg_ServoNorm;
    [SerializeField] private float dbg_StandingStill;
    [SerializeField] private float dbg_Backward;
    [SerializeField] private float dbg_SideIR;
    [SerializeField] private float dbg_Sonar;
    [SerializeField] private float dbg_ActionRate;
    [SerializeField] private float dbg_BallPush;
    [SerializeField] private float dbg_BallSpeed;
    [SerializeField] private int   dbg_HoldTicks;
    [SerializeField] private int   dbg_EpisodeSteps;
    [SerializeField] private float dbg_HintFollow;

    [Header("LiDAR Hint (симуляция подсказки от второго робота)")]
    [SerializeField] private bool  enableLidarHint = true;
    [SerializeField] private float hintAngleNoiseDeg = 12f;
    [SerializeField] private float hintDistNoisePct = 0.15f;
    [SerializeField] private float hintDecaySeconds = 5f;
    [SerializeField] private float hintDecayPerMove = 0.5f;
    [SerializeField] private float hintDecayPerTurn = 1.2f;
    [Range(0f,1f)] [SerializeField] private float hintDropoutChance = 0.25f;
    [SerializeField] private float hintAngleNorm = 90f;

    [Header("Observation Normalization")]
    [SerializeField] private float timeSinceBallCap = 10f;

    // --- Rigidbody y poses de inicio ---
    private Rigidbody rb;
    private Rigidbody ballRb;
    private Vector3 startPos;
    private Quaternion startRot;
    private Quaternion cameraServoBase;
    private float ballStartY;

    // --- Estado interno ---
    private float servoAngle;
    private float lastKnownBallAngle;
    private float timeSinceBall;
    private float hintAngle, hintDistance, hintConfidence;
    private bool  missionDone;

    // --- Éxito graduado ---
    private int  holdTicks;
    private bool touchRewarded;

    // --- Deltas por VISIÓN (no por transform) ---
    private float lastVisionDist = 1f;
    private float lastServoNorm;
    private bool  wasSeeingBallLastStep;
    private bool  wasCloseToBall;
    private int   blindApproachTicks;
    private const int BLIND_APPROACH_MAX = 80;
    private int   lastBallSeenStep = -999;

    // --- Action rate ---
    private float prevGas, prevSteer, prevCam;

    // --- Anti-atasco ---
    private Vector3 lastPosition;
    private int stuckTimer;

    private int episodeSteps;

    public bool IsTraining => Academy.Instance.IsCommunicatorOn;

    // ================= PUBLIC API FOR BrainDebugHUD =================
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
    public float Obs11_DisplacementX     { get; private set; }
    public float Obs12_DisplacementZ     { get; private set; }
    public float Obs13_HeadingNorm       { get; private set; }
    public float Obs14_SpeedNorm         { get; private set; }
    public float Obs15_TimeSinceBallNorm { get; private set; }
    public float Obs16_HintAngle         { get; private set; }
    public float Obs17_HintDistance      { get; private set; }
    public float Obs18_HintConfidence    { get; private set; }

    public float ActGas       { get; private set; }
    public float ActSteer     { get; private set; }
    public float ActCameraCmd { get; private set; }

    public float StepReward       { get; private set; }
    public float CumulativeReward => GetCumulativeReward();
    // ================================================================

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        if (diagLogger == null) diagLogger = GetComponent<DiagnosticLogger>();

        startPos = spawnPoint != null ? spawnPoint.position : transform.position;
        startRot = spawnPoint != null ? spawnPoint.rotation : transform.rotation;

        if (cameraServo != null) cameraServoBase = cameraServo.localRotation;

        if (ball == null && !string.IsNullOrEmpty(ballTag))
        {
            var go = GameObject.FindWithTag(ballTag);
            if (go != null) ball = go.transform;
        }
        if (ball != null)
        {
            ballStartY = ball.position.y;
            ballRb = ball.GetComponent<Rigidbody>();   // нужен для штрафа за СДВИГ
        }
    }

    public override void OnEpisodeBegin()
    {
        episodeSteps = 0;
        ResetEpisodeState();

        if (missionMode)
        {
            CaptureLidarHint();
            return;
        }

        if (obstacleRandomizer != null) obstacleRandomizer.Shuffle();

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        Vector3 spawn = startPos;
        if (robotSpawnJitter > 0f)
            spawn += (startRot * Vector3.right) * Random.Range(-robotSpawnJitter, robotSpawnJitter);

        rb.position = spawn;
        rb.rotation = startRot;   // робот ВСЕГДА смотрит вперёд -> угол подсказки однозначен
        transform.SetPositionAndRotation(spawn, startRot);

        servoAngle = 0f;
        if (cameraServo != null) cameraServo.localRotation = cameraServoBase;

        RespawnBall();

        lastPosition = transform.position;
        CaptureLidarHint();

        if (randomizer != null) randomizer.ApplyEpisodeRandomization(IsTraining);
    }

    private void ResetEpisodeState()
    {
        lastKnownBallAngle = 0f;
        timeSinceBall = timeSinceBallCap;
        lastVisionDist = 1f;
        lastServoNorm = 0f;
        wasSeeingBallLastStep = false;
        wasCloseToBall = false;
        blindApproachTicks = 0;
        lastBallSeenStep = -999;
        prevGas = prevSteer = prevCam = 0f;
        stuckTimer = 0;
        missionDone = false;
        servoAngle = 0f;
        holdTicks = 0;
        touchRewarded = false;
        lastPosition = transform.position;
    }

    private void RespawnBall()
    {
        if (ball == null) return;

        Vector3 fwd  = startRot * Vector3.forward;
        Vector3 side = startRot * Vector3.right;
        float ballR = ball.localScale.x * 0.5f + 0.02f;

        Collider finishCollider = finishArea != null ? finishArea.GetComponent<Collider>() : null;

        Vector3 p = ball.position;
        int guard = 0;
        bool ok = false;

        while (!ok && guard < 40)
        {
            guard++;

            if (finishCollider != null)
            {
                Bounds fb = finishCollider.bounds;
                p = new Vector3(Random.Range(fb.min.x, fb.max.x), ballStartY, Random.Range(fb.min.z, fb.max.z));
            }
            else
            {
                // Curriculum задаёт МАКСИМУМ. Минимум держим ниже него, иначе
                // Random.Range(min > max) ломает "лёгкую" фазу.
                float maxFwd = Academy.Instance.EnvironmentParameters
                                      .GetWithDefault("ball_max_forward", ballMaxForward);
                float minFwd = Mathf.Min(ballMinForward, maxFwd * 0.5f);
                p = startPos
                  + fwd  * Random.Range(minFwd, maxFwd)
                  + side * Random.Range(-ballHalfWidth, ballHalfWidth);
                p.y = ballStartY;
            }

            ok = true;
            foreach (var col in Physics.OverlapSphere(p, ballR))
            {
                if (col.transform == ball) continue;
                if (finishCollider != null && col == finishCollider) continue;
                if (arenaCenter != null && (col.transform == arenaCenter || col.transform.IsChildOf(arenaCenter)))
                    continue;
                ok = false;
                break;
            }
        }

        ball.position = p;

        if (ballRb == null) ballRb = ball.GetComponent<Rigidbody>();
        if (ballRb != null)
        {
            ballRb.isKinematic = false;
            ballRb.linearVelocity = Vector3.zero;
            ballRb.angularVelocity = Vector3.zero;
        }
        var bcol = ball.GetComponent<Collider>();
        if (bcol != null) bcol.enabled = true;
    }

    private void FixedUpdate()
    {
        timeSinceBall += Time.fixedDeltaTime;

        // Спад доверия к подсказке: замороженный вектор устаревает тем быстрее,
        // чем больше робот проехал и повернул (энкодеров нет — не пересчитать).
        if (hintConfidence > 0f)
        {
            float dt = Time.fixedDeltaTime;
            float decay = dt / Mathf.Max(0.1f, hintDecaySeconds)
                        + Mathf.Abs(ActGas)   * dt * hintDecayPerMove
                        + Mathf.Abs(ActSteer) * dt * hintDecayPerTurn;
            hintConfidence = Mathf.Max(0f, hintConfidence - decay);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // --- 1-4: датчики ---
        Obs01_Ultrasonic = sensors != null
            ? Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight) : 1f;
        if (randomizer != null) Obs01_Ultrasonic = randomizer.NoisySonar(Obs01_Ultrasonic, IsTraining);

        Obs02_LeftIR    = sensors != null ? sensors.LeftIR : 0;
        Obs03_RightIR   = sensors != null ? sensors.RightIR : 0;
        Obs04_GripperIR = sensors != null ? sensors.GripperIR : 0;

        // --- 5-8: зрение (реальный YOLO или симулированный) ---
        bool useReal = realVision != null && realVision.useYOLO;
        bool visible  = useReal ? realVision.IsVisible : (yolo != null && yolo.IsVisible);
        float visAngle = useReal ? realVision.RelativeAngle
                                 : (yolo != null ? yolo.RelativeAngle : 0f);
        float visDist  = useReal ? realVision.NormalizedDistance
                                 : (yolo != null ? yolo.NormalizedDistance : 1f);

        if (randomizer != null && rb != null)
            visible = randomizer.FilterBallVisibility(visible, rb.angularVelocity.magnitude, IsTraining);

        Obs08_BallVisible  = visible ? 1f : 0f;
        Obs05_BallAngle    = visible ? visAngle : 0f;
        Obs06_BallDistance = visible ? visDist : 1f;

        if (visible)
        {
            lastKnownBallAngle = visAngle;
            timeSinceBall = 0f;
            lastBallSeenStep = StepCount;
        }
        Obs07_LastKnownAngle = lastKnownBallAngle;

        // --- 9-10 ---
        Obs09_ServoAngleNorm = Mathf.Clamp(servoAngle / Mathf.Max(1f, maxServoAngle), -1f, 1f);
        Obs10_HasBall = Obs04_GripperIR == 1 ? 1f : 0f;

        // --- 11-14: эгоцентрия.
        //     ВНИМАНИЕ: на реальном GFS-X энкодеров и IMU НЕТ. Слоты оставлены
        //     ради Space Size = 18 и совместимости с HUD. После защиты стоит
        //     обнулить их или убрать — это чистый sim2real-риск. ---
        Vector3 displacement = transform.position - startPos;
        Obs11_DisplacementX = Mathf.Clamp(displacement.x / 3f, -1f, 1f);
        Obs12_DisplacementZ = Mathf.Clamp(displacement.z / 3f, -1f, 1f);
        Obs13_HeadingNorm   = Mathf.Repeat(transform.eulerAngles.y, 360f) / 360f;
        Obs14_SpeedNorm     = rb != null ? Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f) : 0f;

        // --- 15 ---
        Obs15_TimeSinceBallNorm = Mathf.Clamp01(timeSinceBall / Mathf.Max(0.1f, timeSinceBallCap));

        // --- 16-18: подсказка. При confidence=0 читается как "информации нет"
        //     (та же конвенция, что и невидимый мяч: угол 0, дистанция 1) ---
        Obs16_HintAngle      = hintAngle * hintConfidence;
        Obs17_HintDistance   = Mathf.Lerp(1f, hintDistance, hintConfidence);
        Obs18_HintConfidence = hintConfidence;

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
        sensor.AddObservation(Obs11_DisplacementX);
        sensor.AddObservation(Obs12_DisplacementZ);
        sensor.AddObservation(Obs13_HeadingNorm);
        sensor.AddObservation(Obs14_SpeedNorm);
        sensor.AddObservation(Obs15_TimeSinceBallNorm);
        sensor.AddObservation(Obs16_HintAngle);
        sensor.AddObservation(Obs17_HintDistance);
        sensor.AddObservation(Obs18_HintConfidence);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        StepReward = 0f;
        episodeSteps++;
        dbg_EpisodeSteps = episodeSteps;

        float gas    = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steer  = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float camCmd = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        ActGas = gas; ActSteer = steer; ActCameraCmd = camCmd;

        if (randomizer != null) randomizer.DelayActions(ref gas, ref steer, ref camCmd);

        // ================================================================
        // ЛИМИТ ЭПИЗОДА — ПРОВЕРЯЕТСЯ САМЫМ ПЕРВЫМ (ФИКС v6.1).
        // В v6.0 он стоял ниже защёлки, та делала return, и эпизод не
        // завершался НИКОГДА: в логе шло "No episode was completed",
        // так утекло 4M шагов и рухнула память.
        // Ни один блок с return не должен стоять выше этой проверки.
        // ================================================================
        // Лимит берём из curriculum: далёкий мяч требует больше времени.
        // При 0.125 м/с и мяче на 2.6 м одна дорога занимает ~21 с — в 1000
        // шагов (20 с) агент физически не успевает доехать.
        int episodeLimit = Mathf.RoundToInt(
            Academy.Instance.EnvironmentParameters.GetWithDefault("episode_length", maxEpisodeSteps));

        if (IsTraining && !missionMode && episodeSteps >= episodeLimit)
        {
            Add(-episodeTimeoutPenalty);
            Academy.Instance.StatsRecorder.Add("Custom/EndTimeout", 1.0f);
            Academy.Instance.StatsRecorder.Add("Custom/ReachSuccess", 0.0f);
            EndEpisode();
            return;
        }

        // ================================================================
        // ФАЗА 0 — МЯЧ МЕЖДУ ГУБКАМИ. Ставим моторы в 0 в ЭТОТ ЖЕ шаг,
        // чтобы робот не толкнул мяч на кадре касания.
        // Награда СТУПЕНЧАТАЯ: касание -> удержание стоя -> полный успех.
        // Первая крошка приходит сразу, поэтому агент не боится доводить
        // манёвр до конца из-за редкости финальной награды.
        // ================================================================
        bool ballRecentlySeen = (StepCount - lastBallSeenStep) < ballSeenWindowSteps;
        bool ballInClaw = Obs04_GripperIR == 1 && (IsTraining || ballRecentlySeen);

        if (ballInClaw)
        {
            missionDone = true;

            if (track != null) track.SetCommand(0f, 0f);
            if (!IsTraining && rosBridge != null) rosBridge.PublishCommand(0f, 0f);

            if (IsTraining && !missionMode)
            {
                if (!touchRewarded) { Add(touchBonus); touchRewarded = true; }

                holdTicks++;
                dbg_HoldTicks = holdTicks;
                Add(holdStillBonus);

                if (holdTicks >= holdStillSteps)
                {
                    Add(reachSuccessReward);
                    Academy.Instance.StatsRecorder.Add("Custom/ReachSuccess", 1.0f);
                    EndEpisode();
                }
            }
            return;
        }
        holdTicks = 0;
        dbg_HoldTicks = 0;

        // Защёлка: доехали, но мяч выкатился — всё равно стоим.
        // ТОЛЬКО В ИНФЕРЕНСЕ. При обучении она замораживала агента навсегда.
        if (latchOnArrival && missionDone && !IsTraining)
        {
            if (track != null) track.SetCommand(0f, 0f);
            if (rosBridge != null) rosBridge.PublishCommand(0f, 0f);
            return;
        }

        // --- Анти-застревание ---
        // ВАЖНО: застреванием считается только попытка ЕХАТЬ без перемещения.
        // Поворот на месте — ЛЕГИТИМНАЯ центровка корпуса, а не застревание.
        if (Mathf.Abs(gas) > 0.15f)
        {
            stuckTimer++;
            if (stuckTimer >= 200)
            {
                if (Vector3.Distance(transform.position, lastPosition) < 0.3f
                    && IsTraining && !missionMode)
                {
                    Add(-stuckPenalty);
                    Academy.Instance.StatsRecorder.Add("Custom/EndStuck", 1.0f);
                    Academy.Instance.StatsRecorder.Add("Custom/ReachSuccess", 0.0f);
                    EndEpisode();
                    return;
                }
                stuckTimer = 0;
                lastPosition = transform.position;
            }
        }
        else { stuckTimer = 0; lastPosition = transform.position; }

        // --- Сервопривод камеры: ГОРИЗОНТАЛЬНЫЙ поворот (рыскание) ---
        servoAngle = Mathf.Clamp(servoAngle + camCmd * servoSpeed * Time.fixedDeltaTime,
                                 -maxServoAngle, maxServoAngle);
        if (cameraServo != null)
        {
            // Ось = мировая вертикаль, переведённая в пространство родителя.
            // Так поворот всегда горизонтальный, независимо от того, как
            // сориентирована импортированная из Blender кость камеры.
            Vector3 yawAxis = cameraServo.parent != null
                ? cameraServo.parent.InverseTransformDirection(Vector3.up)
                : Vector3.up;
            cameraServo.localRotation = Quaternion.AngleAxis(servoAngle, yawAxis) * cameraServoBase;
        }

        // --- Движение ---
        if (track != null) track.SetCommand(gas, steer);
        if (!IsTraining && rosBridge != null)
        {
            rosBridge.PublishCommand(gas, steer);
            rosBridge.PublishCameraCmd(camCmd);
        }

        ComputeRewards(gas, steer, camCmd);

        // --- CSV-логгер для диагностики sim vs real ---
        if (diagLogger != null)
        {
            diagLogger.LogStep(
                StepCount,
                Obs08_BallVisible > 0.5f, Obs05_BallAngle, Obs06_BallDistance,
                Obs01_Ultrasonic, Obs02_LeftIR, Obs03_RightIR, Obs04_GripperIR,
                servoAngle, gas, steer, Obs10_HasBall > 0.5f, holdTicks, false,
                transform.position.x - startPos.x, transform.position.z - startPos.z,
                transform.eulerAngles.y / 360f, rb != null ? rb.linearVelocity.magnitude : 0f
            );
        }
    }

    private void ComputeRewards(float gas, float steer, float camCmd)
    {
        if (!IsTraining || missionMode) return;

        bool  ballVisible = Obs08_BallVisible > 0.5f;
        float currentDist = Obs06_BallDistance;   // НОРМИРОВАННАЯ дистанция от зрения (0..1)
        float camAngle    = Obs05_BallAngle;      // мяч в кадре, от оси КАМЕРЫ
        float servoNorm   = Obs09_ServoAngleNorm; // насколько камера отвёрнута от корпуса
        bool  gripperSees = Obs04_GripperIR == 1;

        dbg_ServoNorm = servoNorm;
        dbg_Approach = 0f;
        dbg_Centering = 0f;
        dbg_AlignFactor = 1f;

        if (ballVisible)
        {
            // Первый кадр после появления мяча — без дельты (иначе спайк)
            if (!wasSeeingBallLastStep)
            {
                lastVisionDist = currentDist;
                lastServoNorm  = servoNorm;
            }

            blindApproachTicks = 0;

            if (wasSeeingBallLastStep)
            {
                // ===== R1: DISTANCE DELTA (пропорциональная) =====
                float delta = lastVisionDist - currentDist;      // >0 = приблизился
                if (Mathf.Abs(delta) < approachSpikeFilter)
                {
                    // Чем ближе мяч — тем важнее точность: 2.0x далеко ... 6.0x вплотную
                    float proximityMultiplier = 2.0f + 4.0f * (1.0f - Mathf.Clamp01(currentDist));

                    // ===== ДВОЙНЫЕ ВОРОТА: "ЕДЕТ ПРЯМО НА МЯЧ" =====
                    // Условие 1: мяч по центру кадра.
                    // Условие 2: камера НЕ отвёрнута сервоприводом.
                    // Оба сразу означают, что КОРПУС смотрит на мяч. Одного
                    // первого мало: робот центровал мяч поворотом камеры и ехал
                    // физически боком — отсюда круги вокруг мяча.
                    float alignFactor = 1f;
                    if (currentDist < alignRequiredDist)
                    {
                        float camCenter = 1f - Mathf.Clamp01(Mathf.Abs(camAngle)
                                                           / Mathf.Max(0.01f, alignTolerance));
                        float servoCenter = 1f - Mathf.Clamp01(Mathf.Abs(servoNorm)
                                                             / Mathf.Max(0.01f, servoTolerance));
                        alignFactor = camCenter * servoCenter;
                    }
                    dbg_AlignFactor = alignFactor;

                    // Отдаление штрафуем ПОЛНОСТЬЮ, без скидки за перекос,
                    // иначе выгодно отъезжать боком (обход ворот).
                    float r = (delta > 0f)
                        ? delta * approachScale * proximityMultiplier * alignFactor
                        : delta * approachScale * proximityMultiplier;
                    Add(r);
                    dbg_Approach = r;
                }

                // ===== ДОВОДКА: возврат камеры к центру при видимом мяче =====
                // Единственный способ вернуть камеру в ноль, не теряя мяч из
                // виду, — довернуть КОРПУС к мячу. Работает на любой дистанции.
                float servoDelta = Mathf.Abs(lastServoNorm) - Mathf.Abs(servoNorm);
                if (Mathf.Abs(servoDelta) < 0.5f)
                {
                    float rc = servoDelta * servoCenteringScale;
                    Add(rc);
                    dbg_Centering = rc;
                }
            }

            // ===== R3: медленный и ровный подъезд =====
            if (currentDist < 0.30f && gas > 0.01f && gas < 0.30f) Add(slowDownBonus);
            if (currentDist < 0.25f && Mathf.Abs(gas) > 0.40f)     Add(-tooFastPenalty);
            // Бонус за "едет прямо": и мяч, и камера по центру
            if (currentDist < 0.40f && Mathf.Abs(camAngle) < 0.15f && Mathf.Abs(servoNorm) < 0.15f)
                Add(alignBonus);

            wasCloseToBall = currentDist <= 0.35f;
            lastVisionDist = currentDist;
            lastServoNorm  = servoNorm;
            wasSeeingBallLastStep = true;
        }
        else
        {
            // Мяч не виден. Если был рядом и ещё не в клешне — он в слепой зоне
            // камеры прямо перед роботом: платим за медленное движение ВПЕРЁД,
            // иначе агент выбирает "безопасный" задний ход и теряет мяч.
            if (wasCloseToBall && !gripperSees)
            {
                blindApproachTicks++;
                if (gas > 0.01f && gas < 0.30f) Add(blindCrawlBonus);
                if (blindApproachTicks >= BLIND_APPROACH_MAX)
                {
                    wasCloseToBall = false;
                    blindApproachTicks = 0;
                }
            }

            // ===== СЛЕДОВАНИЕ ПОДСКАЗКЕ =====
            // Дальность камеры ~0.8 м, а мяч на турнире будет в 2-3 м: слепой
            // участок — ОСНОВНАЯ часть задачи, а весь остальной сигнал живёт
            // внутри if(ballVisible). Без этой награды агент ищет мяч чисто
            // случайно. Платим за движение ВПЕРЁД вдоль вектора подсказки,
            // и только пока доверие к ней живо.
            dbg_HintFollow = 0f;
            if (hintConfidence > 0.05f && gas > 0f)
            {
                float hintErr = Mathf.Abs(Obs16_HintAngle);
                float aligned = 1f - Mathf.Clamp01(hintErr / Mathf.Max(0.01f, hintFollowTolerance));
                float r = aligned * gas * hintFollowScale;
                Add(r);
                dbg_HintFollow = r;
            }

            lastVisionDist = 1f;
            wasSeeingBallLastStep = false;
        }

        // ===== R4: штрафы по ДАТЧИКАМ (есть и на реальном роботе) =====
        dbg_Sonar = 0f;
        if (Obs01_Ultrasonic < sonarThreshold)
        {
            float prox = 1f - (Obs01_Ultrasonic / Mathf.Max(0.001f, sonarThreshold));
            Add(-sonarPenalty * prox);
            dbg_Sonar = -sonarPenalty * prox;
        }

        dbg_SideIR = 0f;
        if (Obs02_LeftIR == 1 || Obs03_RightIR == 1)
        {
            Add(-sideIrPenalty);
            dbg_SideIR = -sideIrPenalty;
        }

        // ===== R5: гладкость (стандарт NVIDIA Isaac Lab) =====
        float actionRate = Mathf.Pow(gas - prevGas, 2)
                         + Mathf.Pow(steer - prevSteer, 2)
                         + Mathf.Pow(camCmd - prevCam, 2);
        Add(-actionRatePenalty * actionRate);
        dbg_ActionRate = -actionRatePenalty * actionRate;

        // Мягкий штраф за задний ход, кроме случая "жмёмся к стене"
        dbg_Backward = 0f;
        if (gas < -0.1f)
        {
            bool nearWall = Obs01_Ultrasonic < sonarThreshold;
            bool nearSide = Obs02_LeftIR == 1 || Obs03_RightIR == 1;
            if (!nearWall && !nearSide)
            {
                Add(-reversePenalty);
                dbg_Backward = -reversePenalty;
            }
        }

        // ===== R7: "ползание" — газ ниже порога считается стоянием =====
        // Закрывает дыру: gas=0.05 -> штрафа нет и движения тоже нет.
        dbg_StandingStill = 0f;
        if (Mathf.Abs(gas) < standingStillGasThreshold)
        {
            Add(-standingStillPenalty);
            dbg_StandingStill = -standingStillPenalty;
        }

        prevGas = gas; prevSteer = steer; prevCam = camCmd;

        // ===== Диагностика в TensorBoard =====
        Academy.Instance.StatsRecorder.Add("Custom/Gas", gas);
        Academy.Instance.StatsRecorder.Add("Custom/IsReverse", gas < -0.1f ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("Custom/BallVisible", Obs08_BallVisible);
        Academy.Instance.StatsRecorder.Add("Custom/HintConf", Obs18_HintConfidence);
        // Метрики центровки пишем ТОЛЬКО при видимом мяче, иначе они мерят
        // холостой скан камеры и превращаются в шум.
        if (ballVisible)
        {
            Academy.Instance.StatsRecorder.Add("Custom/ServoAbs", Mathf.Abs(servoNorm));
            Academy.Instance.StatsRecorder.Add("Custom/CamAngleAbs", Mathf.Abs(camAngle));
        }
    }

    /// <summary>
    /// Анти-снегоуборщик v6: штрафуем не КАСАНИЕ, а СДВИГ мяча.
    ///
    /// Плотный штраф за любой контакт усваивается быстрее редкой награды за
    /// успех, и агент выучивает "рядом с мячом = боль" раньше, чем "мяч в
    /// клешне = награда" — то есть боится подъезжать вплотную. Физически же
    /// запретить надо именно ТОЛЧОК: аккуратно коснуться, не сдвинув мяч,
    /// это ровно то поведение, которое нам нужно.
    /// </summary>
    private void OnCollisionStay(Collision c)
    {
        dbg_BallPush = 0f;
        if (!IsTraining || missionMode) return;
        if (!c.collider.CompareTag(ballTag)) return;
        if (Obs04_GripperIR == 1) return;   // контакт в зоне клешни легитимен

        float ballSpeed = ballRb != null ? ballRb.linearVelocity.magnitude : 1f;
        dbg_BallSpeed = ballSpeed;
        if (ballSpeed < ballPushSpeedThreshold) return;   // лёгкое касание — без штрафа

        float k = Mathf.Clamp01(ballSpeed / Mathf.Max(0.01f, ballPushSpeedNorm));
        Add(-ballPushPenalty * k);
        dbg_BallPush = -ballPushPenalty * k;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;

        float gas = 0f, steer = 0f, cam = 0f;

        bool  sees      = Obs08_BallVisible > 0.5f;
        float camAngle  = Obs05_BallAngle;
        float dist      = Obs06_BallDistance;
        float servoNorm = Obs09_ServoAngleNorm;

        if (Obs04_GripperIR == 1)
        {
            gas = 0f; steer = 0f;                       // мяч между губками -> стоп
        }
        else if (!sees)
        {
            // Мяч рождается ВПЕРЕДИ и обычно просто заслонён препятствием.
            // Разворот на месте бесполезен — надо ЕХАТЬ и объезжать.
            float t = Time.time;
            cam = Mathf.Sin(t * 1.2f) * 0.9f;           // широкий скан камерой

            if (Obs18_HintConfidence > 0.05f)
            {
                steer = Mathf.Clamp(Obs16_HintAngle * 1.5f, -1f, 1f);
                gas = 0.35f;
            }
            else
            {
                steer = Mathf.Sin(t * 0.25f) * 0.4f;
                gas = 0.30f;
            }

            // Объезд препятствия по датчикам
            if (Obs01_Ultrasonic < 0.15f)      { gas = 0.10f; steer =  0.7f; }
            else if (Obs02_LeftIR == 1)        { steer =  0.5f; }
            else if (Obs03_RightIR == 1)       { steer = -0.5f; }
        }
        else
        {
            // Камеру ВОЗВРАЩАЕМ к центру, а доворачиваем КОРПУСОМ.
            cam = Mathf.Lerp(servoNorm, 0f, Time.deltaTime * 3f);
            float turn = camAngle + servoNorm;          // куда доворачивать корпус

            if (Mathf.Abs(turn) > 0.15f)
            {
                gas = dist < alignRequiredDist ? 0.05f : 0.25f;
                steer = Mathf.Clamp(turn * 0.8f, -1f, 1f);
            }
            else if (dist > 0.16f) { gas = 0.45f; steer = turn * 0.5f; }
            else                   { gas = 0.20f; steer = turn * 0.5f; }
        }

        if (cont.Length > 0) cont[0] = gas;
        if (cont.Length > 1) cont[1] = steer;
        if (cont.Length > 2) cont[2] = cam;

        // Аварийный перехват: зажми LEFT SHIFT для ручного WASD
        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (cont.Length > 0) cont[0] = Input.GetAxis("Vertical");
            if (cont.Length > 1) cont[1] = Input.GetAxis("Horizontal");
            if (cont.Length > 2)
            {
                if (Input.GetKey(KeyCode.Q)) cont[2] = -1f;
                else if (Input.GetKey(KeyCode.E)) cont[2] = 1f;
            }
        }
    }

    /// <summary>
    /// Замер "ЛиДАРа" в момент старта эпизода. Робот всегда стартует лицом
    /// вперёд, поэтому угол однозначен. Значение ЗАМОРАЖИВАЕТСЯ: одометрии
    /// нет, обновлять его нечем — можно только терять доверие.
    /// </summary>
    private void CaptureLidarHint()
    {
        hintAngle = 0f; hintDistance = 1f; hintConfidence = 0f;
        if (!enableLidarHint || ball == null) return;
        if (Random.value < hintDropoutChance) return;   // эпизод без подсказки

        Vector3 to = ball.position - transform.position; to.y = 0f;

        float ang = Vector3.SignedAngle(transform.forward, to, Vector3.up)
                  + Random.Range(-hintAngleNoiseDeg, hintAngleNoiseDeg);
        hintAngle = Mathf.Clamp(ang / Mathf.Max(1f, hintAngleNorm), -1f, 1f);

        float d = to.magnitude * (1f + Random.Range(-hintDistNoisePct, hintDistNoisePct));
        hintDistance = Mathf.Clamp01(d / 3.5f);

        hintConfidence = 1f;
    }

    private void Add(float r) { AddReward(r); StepReward += r; }
}
