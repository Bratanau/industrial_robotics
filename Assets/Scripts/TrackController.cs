using UnityEngine;

/// <summary>
/// Контроллер гусеничного робота с дифференциальным приводом.
/// Принимает команды газа (gas) и руля (steer) в диапазоне [-1, 1],
/// смешивает их в скорости левого/правого борта, эмулирует поведение реальных
/// моторов (PWM, мёртвая зона, минимальный старт, плавный разгон) и двигает
/// Rigidbody через MovePosition/MoveRotation в FixedUpdate.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("Базовые скорости")]
    [Tooltip("Базовая линейная скорость робота, м/с")]
    [SerializeField] private float moveSpeed = 0.57f;

    [Tooltip("Базовая скорость поворота, град/с")]
    [SerializeField] private float turnSpeed = 120f;

    [Tooltip("Коэффициент влияния руля на скорости бортов")]
    [SerializeField] private float turnK = 0.30f;

    [Header("Лимиты")]
    [Tooltip("Лимит поступательной скорости, м/с")]
    [SerializeField] private float maxLinearCmd = 0.25f;

    [Tooltip("Мёртвая зона PWM в процентах — ниже неё моторы стоят")]
    [SerializeField] private float motorDeadzone = 10f;

    [Tooltip("Минимальный стартовый PWM, если сигнал выше мёртвой зоны")]
    [SerializeField] private float minMotorPwm = 35f;

    //[Tooltip("Максимальное изменение PWM за один физический тик (плавный разгон)")]
    //[SerializeField] private float maxPwmStep = 15f;

    [Header("Калибровка м/с → PWM")]
    [Tooltip("Масштабный коэффициент перехода м/с → PWM (%). Референс = 200")]
    [SerializeField] private float speedToPwm = 200f;

    [Header("Вход (для теста / внешнего управления)")]
    [Tooltip("Газ в диапазоне [-1, 1]. Можно задавать снаружи (нейросеть, ROS и т.п.)")]
    [Range(-1f, 1f)] public float gas;

    [Tooltip("Руль в диапазоне [-1, 1]")]
    [Range(-1f, 1f)] public float steer;

    [Tooltip("Читать ли газ/руль с клавиатуры (WASD/стрелки) для отладки")]
    [SerializeField] private bool useKeyboardInput = true;

    // --- внутреннее состояние ---
    private Rigidbody rb;
    private float currentLeftPwm;
    private float currentRightPwm;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();

        // Настройки стабильности физики
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.linearDamping = 8f;   // в Unity < 6 называется rb.drag
        rb.angularDamping = 10f; // в Unity < 6 называется rb.angularDrag
    }

    private void Update()
    {
        if (useKeyboardInput)
        {
            gas = Input.GetAxis("Vertical");
            steer = Input.GetAxis("Horizontal");
        }
    }

    private void FixedUpdate()
    {
        // 1. Смешиваем газ и руль в целевые скорости бортов (м/с)
        //    linear идёт на оба борта одинаково, angular — с разными знаками
        float linear = Mathf.Clamp(gas * moveSpeed, -maxLinearCmd, maxLinearCmd);
        float angular = steer * turnK * moveSpeed;

        float leftTargetSpeed  = linear - angular;
        float rightTargetSpeed = linear + angular;

        // 2. Переводим м/с -> сырой PWM (%)
        float leftPwmRaw  = leftTargetSpeed  * speedToPwm;
        float rightPwmRaw = rightTargetSpeed * speedToPwm;

        // 3. Мёртвая зона + минимальный старт
        float leftPwmTarget  = ApplyDeadzone(leftPwmRaw);
        float rightPwmTarget = ApplyDeadzone(rightPwmRaw);

	// 4
	currentLeftPwm  = leftPwmTarget;
	currentRightPwm = rightPwmTarget;

        // 5. Обратно PWM -> эффективная м/с для каждой гусеницы
        float leftEff  = currentLeftPwm  / speedToPwm;
        float rightEff = currentRightPwm / speedToPwm;

        // 6. Из скоростей бортов получаем линейную и угловую скорость робота
        float linearOut = (leftEff + rightEff) * 0.5f;

        //    angular = steer * turnK * moveSpeed  =>  steer = (right - left) / (2 * turnK * moveSpeed)
        float steerEff = 0f;
        if (turnK > 0.0001f && moveSpeed > 0.0001f)
        {
            steerEff = (rightEff - leftEff) / (2f * turnK * moveSpeed);
            steerEff = Mathf.Clamp(steerEff, -1f, 1f);
        }
        float angularOutDegPerSec = steerEff * turnSpeed;

        // 7. Применяем к Rigidbody
        Vector3 deltaMove = transform.forward * linearOut * Time.fixedDeltaTime;
        Quaternion deltaTurn = Quaternion.Euler(0f, angularOutDegPerSec * Time.fixedDeltaTime, 0f);

        rb.MovePosition(rb.position + deltaMove);
        rb.MoveRotation(rb.rotation * deltaTurn);
    }

    /// <summary>
    /// Мёртвая зона и минимальный стартовый порог моторов.
    /// Ниже motorDeadzone — 0. Между motorDeadzone и minMotorPwm — подтягиваем к minMotorPwm.
    /// Значение обрезаем в пределах [-100, 100] %.
    /// </summary>
    private float ApplyDeadzone(float pwm)
    {
        float abs = Mathf.Abs(pwm);
        if (abs < motorDeadzone) return 0f;
        if (abs < minMotorPwm)   return Mathf.Sign(pwm) * minMotorPwm;
        return Mathf.Clamp(pwm, -100f, 100f);
    }

    /// <summary>
    /// Внешний вход (например, от нейросети или ROS-моста).
    /// Значения будут автоматически ограничены [-1, 1].
    /// </summary>
    public void SetCommand(float gasCmd, float steerCmd)
    {
        gas   = Mathf.Clamp(gasCmd,   -1f, 1f);
        steer = Mathf.Clamp(steerCmd, -1f, 1f);
    }

    /// <summary>External setter for Domain Randomization (Practice 5).</summary>
    public void SetMotorParams(float newMoveSpeed, float newTurnSpeed)
    {
        moveSpeed = newMoveSpeed;
        turnSpeed = newTurnSpeed;
    }
}
