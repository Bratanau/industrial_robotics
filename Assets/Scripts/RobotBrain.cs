using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/// <summary>
/// ИИ-агент робота на базе ML-Agents (15 наблюдений, 3 непрер. + 1 дискр. действие).
/// Наблюдения хранятся как публичные поля/свойства — доступны из HUD и Inspector
/// (в Debug-режиме Inspector). Порядок наблюдений синхронизирован со спецификацией.
/// </summary>
public class RobotBrain : Agent
{
    [Header("Ссылки на компоненты робота")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private GripperController gripper;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraServo;   // может быть null, если камера не вращается
    [SerializeField] private Rigidbody rb;

    [Header("Цель и арена")]
    [SerializeField] private Transform ballTransform;
    [SerializeField] private Vector3 arenaMin = new Vector3(-10f, -10f, -10f);
    [SerializeField] private Vector3 arenaMax = new Vector3( 10f,  10f,  10f);

    [Tooltip("Сбрасывать ли мяч в новую позицию при старте эпизода. Сними для отладки.")]
    [SerializeField] private bool resetBallOnEpisode = true;

    [Tooltip("Если resetBallOnEpisode = true — рандомизировать позицию (иначе фиксированная точка перед роботом).")]
    [SerializeField] private bool randomizeBallOnReset = true;

    [Tooltip("Минимальная дистанция спавна мяча от робота (только для рандомизации), м")]
    [SerializeField] private float ballSpawnMinDistance = 0.5f;

    [Tooltip("Максимальная дистанция спавна мяча от робота (только для рандомизации), м")]
    [SerializeField] private float ballSpawnMaxDistance = 1.5f;

    [Header("Сервопривод камеры (если есть)")]
    [SerializeField] private float servoMaxAngle = 90f;
    [SerializeField] private float servoSpeed    = 60f;

    [Header("Награды")]
    [SerializeField] private float approachRewardScale    = 0.05f;
    [SerializeField] private float actionRatePenaltyScale = 0.001f;
    [SerializeField] private float centeringBonusScale    = 0.01f;
    [SerializeField] private float wallProximityPenalty   = 0.02f;
    [SerializeField] private float successReward          = 5.0f;
    [SerializeField] private float outOfBoundsPenalty     = -2.0f;
    [SerializeField] private float wallCriticalUsThreshold = 0.15f;

    // ---------- Наблюдения (Debug — видны в Inspector, доступны через свойства) ----------
    [Header("OBSERVATIONS (read-only, для отладки)")]
    [SerializeField] private float o01_ultrasonic;
    [SerializeField] private int   o02_leftIR;
    [SerializeField] private int   o03_rightIR;
    [SerializeField] private int   o04_gripperIR;
    [SerializeField] private float o05_ballAngle;
    [SerializeField] private float o06_ballDistance;
    [SerializeField] private float o07_lastKnownAngle;
    [SerializeField] private float o08_ballVisible;
    [SerializeField] private float o09_servoAngleNorm;
    [SerializeField] private float o10_hasBall;
    [SerializeField] private float o11_dxNorm;
    [SerializeField] private float o12_dzNorm;
    [SerializeField] private float o13_headingNorm;
    [SerializeField] private float o14_speed;
    [SerializeField] private float o15_timeSinceBallNorm;

    [Header("ACTIONS (read-only, для отладки)")]
    [SerializeField] private float actGas;
    [SerializeField] private float actSteer;
    [SerializeField] private float actCameraCmd;
    [SerializeField] private int   actGripCmd;

    [Header("REWARD (read-only)")]
    [SerializeField] private float rewardStep;         // награда за последний шаг
    [SerializeField] private float rewardCumulative;   // накопленная за эпизод

    // Публичные аксессоры для HUD
    public float Obs01_Ultrasonic       => o01_ultrasonic;
    public int   Obs02_LeftIR           => o02_leftIR;
    public int   Obs03_RightIR          => o03_rightIR;
    public int   Obs04_GripperIR        => o04_gripperIR;
    public float Obs05_BallAngle        => o05_ballAngle;
    public float Obs06_BallDistance     => o06_ballDistance;
    public float Obs07_LastKnownAngle   => o07_lastKnownAngle;
    public float Obs08_BallVisible      => o08_ballVisible;
    public float Obs09_ServoAngleNorm   => o09_servoAngleNorm;
    public float Obs10_HasBall          => o10_hasBall;
    public float Obs11_DxNorm           => o11_dxNorm;
    public float Obs12_DzNorm           => o12_dzNorm;
    public float Obs13_HeadingNorm      => o13_headingNorm;
    public float Obs14_Speed            => o14_speed;
    public float Obs15_TimeSinceBallNorm=> o15_timeSinceBallNorm;

    public float ActGas        => actGas;
    public float ActSteer      => actSteer;
    public float ActCameraCmd  => actCameraCmd;
    public int   ActGripCmd    => actGripCmd;

    public float StepReward       => rewardStep;
    public float CumulativeReward => rewardCumulative;

    // ---- Внутреннее состояние ----
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float lastKnownBallAngle;
    private float timeSinceLastBallSeen;
    private float servoAngle;
    private float prevDistanceToBall;
    private float prevGas, prevSteer;
    private bool  hasBall;
    private float arenaSize;

    private bool initialized;

    private void Awake()
    {
        EnsureInitialized();
    }

    public override void Initialize()
    {
        EnsureInitialized();
    }

    private void EnsureInitialized()
    {
        if (initialized) return;
        if (rb == null) rb = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;
        arenaSize = Mathf.Max(1f, Mathf.Max(arenaMax.x - arenaMin.x, arenaMax.z - arenaMin.z));
        initialized = true;
    }

    // FixedUpdate работает независимо от ML-Agents pipeline —
    // обеспечивает актуальные значения в HUD/Inspector даже если
    // Behavior Parameters/Decision Requester ещё не настроены.
    private void FixedUpdate()
    {
        // Обновляем "память" про мяч
        if (yoloCamera != null && yoloCamera.IsVisible)
        {
            lastKnownBallAngle = yoloCamera.RelativeAngle;
            timeSinceLastBallSeen = 0f;
        }
        else
        {
            timeSinceLastBallSeen += Time.fixedDeltaTime;
        }
        hasBall = gripper != null && gripper.IsHolding;

        // Пересчитываем все 15 наблюдений
        ComputeObservations();
    }

    public override void OnEpisodeBegin()
    {
        transform.SetPositionAndRotation(startPosition, startRotation);
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        if (gripper != null && gripper.IsHolding) gripper.ReleaseCommand();

        servoAngle = 0f;
        if (cameraServo != null) cameraServo.localRotation = Quaternion.identity;

        // Сброс мяча
        if (ballTransform != null && resetBallOnEpisode)
        {
            Vector3 newBallPos;

            if (randomizeBallOnReset)
            {
                // Случайное направление и дистанция от робота
                float angle = Random.Range(0f, Mathf.PI * 2f);
                float dist  = Random.Range(ballSpawnMinDistance, ballSpawnMaxDistance);
                Vector3 offset = new Vector3(Mathf.Sin(angle) * dist, 0f, Mathf.Cos(angle) * dist);
                newBallPos = startPosition + offset;

                // Ограничиваем аренной, чтобы не улетел за пределы
                newBallPos.x = Mathf.Clamp(newBallPos.x, arenaMin.x + 0.2f, arenaMax.x - 0.2f);
                newBallPos.z = Mathf.Clamp(newBallPos.z, arenaMin.z + 0.2f, arenaMax.z - 0.2f);
                newBallPos.y = startPosition.y;
            }
            else
            {
                // Фиксированная точка — метр перед стартовой позицией робота
                newBallPos = startPosition + startRotation * new Vector3(0f, 0f, 1f);
            }

            ballTransform.position = newBallPos;

            var ballRb = ballTransform.GetComponent<Rigidbody>();
            if (ballRb != null) { ballRb.linearVelocity = Vector3.zero; ballRb.angularVelocity = Vector3.zero; }
        }

        lastKnownBallAngle = 0f;
        timeSinceLastBallSeen = 0f;
        prevGas = 0f;
        prevSteer = 0f;
        hasBall = false;
        rewardStep = 0f;
        rewardCumulative = 0f;

        prevDistanceToBall = ballTransform != null
            ? Vector3.Distance(transform.position, ballTransform.position)
            : 0f;
    }

    /// <summary>Обновляет все 15 полей наблюдений (можно вызывать из HUD или наградного расчёта).</summary>
    private void ComputeObservations()
    {
        // 1. УЗ (мин из двух)
        o01_ultrasonic = sensors != null
            ? Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight)
            : 1f;

        // 2, 3. Боковые ИК
        o02_leftIR  = sensors != null ? sensors.LeftIR  : 0;
        o03_rightIR = sensors != null ? sensors.RightIR : 0;

        // 4. ИК клешни
        o04_gripperIR = sensors != null ? sensors.GripperIR : 0;

        // 5, 6. Угол и дистанция до мяча (по камере)
        bool visible = yoloCamera != null && yoloCamera.IsVisible;
        o05_ballAngle    = visible ? yoloCamera.RelativeAngle : 0f;
        o06_ballDistance = visible ? yoloCamera.NormalizedDistance : 1f;

        // 7. Последнее известное направление
        o07_lastKnownAngle = lastKnownBallAngle;

        // 8. Флаг видимости
        o08_ballVisible = visible ? 1f : 0f;

        // 9. Угол сервопривода (нормализованный)
        o09_servoAngleNorm = servoAngle / Mathf.Max(1f, servoMaxAngle);

        // 10. hasBall
        o10_hasBall = hasBall ? 1f : 0f;

        // 11, 12. Смещение от старта (нормализованное к размеру арены)
        Vector3 delta = transform.position - startPosition;
        o11_dxNorm = delta.x / arenaSize;
        o12_dzNorm = delta.z / arenaSize;

        // 13. Heading
        float headingDeg = Mathf.DeltaAngle(startRotation.eulerAngles.y, transform.eulerAngles.y);
        o13_headingNorm = headingDeg / 180f;

        // 14. Скорость
        o14_speed = rb != null ? rb.linearVelocity.magnitude : 0f;

        // 15. Время с последней детекции (нормализованное к 10 сек)
        o15_timeSinceBallNorm = Mathf.Clamp(timeSinceLastBallSeen, 0f, 10f) / 10f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        ComputeObservations();

        sensor.AddObservation(o01_ultrasonic);
        sensor.AddObservation(o02_leftIR);
        sensor.AddObservation(o03_rightIR);
        sensor.AddObservation(o04_gripperIR);
        sensor.AddObservation(o05_ballAngle);
        sensor.AddObservation(o06_ballDistance);
        sensor.AddObservation(o07_lastKnownAngle);
        sensor.AddObservation(o08_ballVisible);
        sensor.AddObservation(o09_servoAngleNorm);
        sensor.AddObservation(o10_hasBall);
        sensor.AddObservation(o11_dxNorm);
        sensor.AddObservation(o12_dzNorm);
        sensor.AddObservation(o13_headingNorm);
        sensor.AddObservation(o14_speed);
        sensor.AddObservation(o15_timeSinceBallNorm);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        actGas       = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        actSteer     = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        actCameraCmd = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        actGripCmd   = actions.DiscreteActions[0];

        if (trackController != null) trackController.SetCommand(actGas, actSteer);

        // Сервопривод — только если назначен
        if (cameraServo != null)
        {
            servoAngle = Mathf.Clamp(
                servoAngle + actCameraCmd * servoSpeed * Time.fixedDeltaTime,
                -servoMaxAngle, servoMaxAngle);
            cameraServo.localRotation = Quaternion.Euler(0f, servoAngle, 0f);
        }

        if (gripper != null)
        {
            if (actGripCmd == 1 && !gripper.IsHolding) gripper.GripCommand();
            else if (actGripCmd == 2 && gripper.IsHolding) gripper.ReleaseCommand();
        }

        // Состояние (lastKnownBallAngle, timeSinceLastBallSeen, hasBall) и
        // ComputeObservations обновляются в FixedUpdate независимо от pipeline.

        // Награды с отслеживанием
        float before = GetCumulativeReward();
        ApplyRewards(actGas, actSteer);
        float after = GetCumulativeReward();
        rewardStep = after - before;
        rewardCumulative = after;

        prevGas = actGas;
        prevSteer = actSteer;
    }

    private void ApplyRewards(float gas, float steer)
    {
        if (ballTransform == null) return;

        float currentDistance = Vector3.Distance(transform.position, ballTransform.position);

        float delta = prevDistanceToBall - currentDistance;
        float proximityFactor = Mathf.Clamp01(1f - currentDistance / 2f);
        AddReward(delta * approachRewardScale * (1f + proximityFactor));
        prevDistanceToBall = currentDistance;

        float actionDiff = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        AddReward(-actionDiff * actionRatePenaltyScale);

        if (yoloCamera != null && yoloCamera.IsVisible)
            AddReward((1f - Mathf.Abs(yoloCamera.RelativeAngle)) * centeringBonusScale);

        if (sensors != null)
        {
            float minUs = Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight);
            if (minUs < wallCriticalUsThreshold) AddReward(-wallProximityPenalty);
            if (sensors.LeftIR == 1 || sensors.RightIR == 1) AddReward(-wallProximityPenalty);
        }

        if (hasBall) { AddReward(successReward); EndEpisode(); return; }

        Vector3 pos = transform.position;
        if (pos.x < arenaMin.x || pos.x > arenaMax.x ||
            pos.y < arenaMin.y || pos.y > arenaMax.y ||
            pos.z < arenaMin.z || pos.z > arenaMax.z)
        {
            AddReward(outOfBoundsPenalty);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuous = actionsOut.ContinuousActions;
        var discrete   = actionsOut.DiscreteActions;

        continuous[0] = Input.GetAxis("Vertical");
        continuous[1] = Input.GetAxis("Horizontal");

        float camCmd = 0f;
        if (Input.GetKey(KeyCode.Q)) camCmd -= 1f;
        if (Input.GetKey(KeyCode.E)) camCmd += 1f;
        continuous[2] = camCmd;

        int grip = 0;
        if (Input.GetKeyDown(KeyCode.Space))     grip = 1;
        if (Input.GetKeyDown(KeyCode.LeftShift)) grip = 2;
        discrete[0] = grip;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1f, 0.6f, 0f, 0.5f);
        Vector3 center = (arenaMin + arenaMax) * 0.5f;
        Vector3 size = arenaMax - arenaMin;
        Gizmos.DrawWireCube(center, size);
    }
}