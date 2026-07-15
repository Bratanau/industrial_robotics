using UnityEngine;

/// <summary>
/// Симуляция сенсоров робота через Physics.Raycast.
/// - Два УЗ датчика (левый и правый): веер лучей в конусе ~30°, минимальная дистанция, игнор мяча.
/// - Три ИК датчика препятствий (лево, центр, право): короткий луч, 1 если стена рядом, 0 если пусто.
/// - ИК датчик клешни: короткий луч внутрь захвата, реагирует только на объект с тегом мяча.
/// Все лучи стреляют по локальной оси forward соответствующей точки-якоря —
/// поэтому просто разверни каждую точку в редакторе так, чтобы её синяя стрелка
/// смотрела туда, куда должен «смотреть» датчик.
/// </summary>
public class VirtualSensors : MonoBehaviour
{
    [Header("УЗ датчики (Empty GameObject'ы)")]
    [SerializeField] private Transform leftSuperSonic;   // левый УЗ
    [SerializeField] private Transform rightSuperSonic;  // правый УЗ

    [Header("ИК препятствий")]
    [SerializeField] private Transform leftIRPoint;      // ИК слева
    [SerializeField] private Transform centerIRPoint;    // ИК по центру
    [SerializeField] private Transform rightIRPoint;     // ИК справа

    [Header("ИК клешни (опционально)")]
    [SerializeField] private Transform gripperIRPoint;   // ИК внутрь захвата

    [Header("Настройки ультразвука")]
    [Tooltip("Максимальная дистанция УЗ, м")]
    [SerializeField] private float ultrasonicMaxDistance = 2.0f;
    [Tooltip("Полный угол конуса обзора, градусов")]
    [SerializeField] private float ultrasonicConeAngle = 30f;
    [Tooltip("Количество лучей в веере (нечётное — центральный + симметричные)")]
    [SerializeField] private int ultrasonicRayCount = 5;

    [Header("Настройки ИК препятствий")]
    [Tooltip("Дальность ИК препятствий, м (реальные ~15 см)")]
    [SerializeField] private float irObstacleDistance = 0.15f;

    [Header("Настройки ИК клешни")]
    [Tooltip("Дальность ИК клешни, м (реальные ~7-8 см)")]
    [SerializeField] private float gripperIRDistance = 0.08f;

    [Header("Layers / tags")]
    [Tooltip("Слои, которые считаются препятствиями/стенами")]
    public LayerMask obstacleMask = ~0; // everything by default
    [Tooltip("Тег мяча — УЗ его игнорирует, ИК клешни наоборот ищет именно его")]
    public string ballTag = "TargetBall";

    [Header("Отладка")]
    [SerializeField] private bool drawGizmos = true;

    // ---- Публичные показания датчиков ----
    /// <summary>Левый УЗ: 0 — вплотную, 1 — чисто.</summary>
    public float UltrasonicLeft { get; private set; } = 1f;
    /// <summary>Правый УЗ: 0 — вплотную, 1 — чисто.</summary>
    public float UltrasonicRight { get; private set; } = 1f;

    /// <summary>1 если слева близко стена, иначе 0.</summary>
    public int LeftIR { get; private set; }
    /// <summary>1 если по центру близко стена, иначе 0.</summary>
    public int CenterIR { get; private set; }
    /// <summary>1 если справа близко стена, иначе 0.</summary>
    public int RightIR { get; private set; }

    /// <summary>1 если в клешне мяч, иначе 0.</summary>
    public int GripperIR { get; private set; }

    /// <summary>Коллайдер мяча, обнаруженного ИК клешни на этом кадре (null, если мяча нет).</summary>
    public Collider GripperDetectedBall { get; private set; }

    private void Update()
    {
        UltrasonicLeft = ReadUltrasonic(leftSuperSonic);
        UltrasonicRight = ReadUltrasonic(rightSuperSonic);

        LeftIR = ReadObstacleIR(leftIRPoint, irObstacleDistance);
        CenterIR = ReadObstacleIR(centerIRPoint, irObstacleDistance);
        RightIR = ReadObstacleIR(rightIRPoint, irObstacleDistance);

        GripperIR = ReadGripperIR();
    }

    /// <summary>
    /// Веер лучей в конусе ultrasonicConeAngle, ищем ближайшее препятствие,
    /// игнорируя мяч (реальный УЗ слишком грубый, чтобы его видеть).
    /// </summary>
    private float ReadUltrasonic(Transform anchor)
    {
        if (anchor == null) return 1f;

        float closestDistance = ultrasonicMaxDistance;
        bool hitSomething = false;

        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfAngle = ultrasonicConeAngle * 0.5f;

        for (int i = 0; i < rays; i++)
        {
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-halfAngle, halfAngle, t);

            Vector3 dir = Quaternion.AngleAxis(angle, anchor.up) * anchor.forward;

            // RaycastAll, чтобы можно было пропустить мяч и упереться в стену за ним.
            RaycastHit[] hits = Physics.RaycastAll(anchor.position, dir, ultrasonicMaxDistance, obstacleMask);
            System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

            foreach (var hit in hits)
            {
                if (hit.collider.CompareTag(ballTag))
                {
                    continue; // ультразвук не видит маленький мяч
                }

                hitSomething = true;
                if (hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                }
                break; // первое непустое попадание после фильтрации мяча — уже ближайшее
            }
        }

        if (!hitSomething)
        {
            return 1f;
        }

        return Mathf.Clamp01(closestDistance / ultrasonicMaxDistance);
    }

    /// <summary>
    /// Одиночный короткий луч для ИК-датчиков препятствий.
    /// Возвращает 1, если стена обнаружена в пределах range, иначе 0.
    /// </summary>
    private int ReadObstacleIR(Transform origin, float range)
    {
        if (origin == null) return 0;

        if (Physics.Raycast(origin.position, origin.forward, out RaycastHit hit, range, obstacleMask))
        {
            return 1;
        }

        return 0;
    }

    /// <summary>
    /// Детектирует мяч на близкой дистанции внутри клешни.
    /// Возвращает 1, если мяч (по тегу) обнаружен в пределах gripperIRDistance, иначе 0.
    /// </summary>
    private int ReadGripperIR()
    {
        if (gripperIRPoint == null)
        {
            GripperDetectedBall = null;
            return 0;
        }

        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, gripperIRDistance, obstacleMask))
        {
            if (hit.collider.CompareTag(ballTag))
            {
                GripperDetectedBall = hit.collider;
                return 1;
            }
        }

        GripperDetectedBall = null;
        return 0;
    }

    // ---- Визуализация в редакторе ----
    private void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        // УЗ конусы — левый и правый
        Gizmos.color = Color.cyan;
        DrawUltrasonicCone(leftSuperSonic);
        DrawUltrasonicCone(rightSuperSonic);

        // ИК препятствий
        Gizmos.color = Color.yellow;
        if (leftIRPoint != null) Gizmos.DrawRay(leftIRPoint.position, leftIRPoint.forward * irObstacleDistance);
        if (centerIRPoint != null) Gizmos.DrawRay(centerIRPoint.position, centerIRPoint.forward * irObstacleDistance);
        if (rightIRPoint != null) Gizmos.DrawRay(rightIRPoint.position, rightIRPoint.forward * irObstacleDistance);

        // ИК клешни
        Gizmos.color = Color.magenta;
        if (gripperIRPoint != null) Gizmos.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperIRDistance);
    }

    private void DrawUltrasonicCone(Transform anchor)
    {
        if (anchor == null) return;

        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfCone = ultrasonicConeAngle * 0.5f;
        for (int i = 0; i < rays; i++)
        {
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-halfCone, halfCone, t);
            Vector3 dir = Quaternion.AngleAxis(angle, anchor.up) * anchor.forward;
            Gizmos.DrawRay(anchor.position, dir * ultrasonicMaxDistance);
        }
    }
}