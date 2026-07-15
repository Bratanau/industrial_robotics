/// <summary>
/// Симуляция бортовой YOLO-камеры без реального рендеринга.
/// Проецирует 3D-позицию мяча в 2D viewport, проверяет FOV и линию видимости
/// (пропускает собственные коллайдеры робота).
/// </summary>
/// 
/// 
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Ссылки")]
    [Tooltip("Камера робота. Если пусто — ищется в этом объекте или его детях")]
    [SerializeField] private Camera cam;

    [Tooltip("Целевой мяч. Если пусто — ищется по тегу")]
    [SerializeField] private Transform targetBall;

    [Tooltip("Корневой объект робота, чьи коллайдеры игнорировать. Если пусто — берётся transform.root")]
    [SerializeField] private Transform robotRoot;

    [SerializeField] private string ballTag = "TargetBall";

    [Header("Параметры камеры")]
    [Tooltip("Горизонтальный FOV в градусах")]
    [SerializeField] private float hFOV = 40f;

    [Tooltip("Максимальная дальность видимости, м")]
    [SerializeField] private float maxDistance = 2.0f;

    [Header("Отладка")]
    [SerializeField] private bool drawGizmos = true;
    [Tooltip("Печатать в Console причину, почему мяч НЕ виден (каждые ~2 сек)")]
    [SerializeField] private bool verboseLogging = false;

    // ---- Debug view (только для чтения в Inspector) ----
    [Header("Debug (read-only)")]
    [SerializeField] private float dbg_distanceToBall;
    [SerializeField] private float dbg_angleToBall;
    [SerializeField] private string dbg_lastReason = "-";

    // ---- Публичные показания ----
    public bool  IsVisible          { get; private set; }
    public float RelativeAngle      { get; private set; }
    public float NormalizedDistance { get; private set; } = 1f;

    private float lastLogTime;

    private void Awake()
    {
        if (cam == null) cam = GetComponent<Camera>();
        if (cam == null) cam = GetComponentInChildren<Camera>();
        if (robotRoot == null) robotRoot = transform.root;
    }

    private void FixedUpdate()
    {
        IsVisible = false;
        RelativeAngle = 0f;
        NormalizedDistance = 1f;

        // --- Ищем камеру, если не назначена ---
        if (cam == null)
        {
            cam = GetComponent<Camera>();
            if (cam == null) cam = GetComponentInChildren<Camera>();
            if (cam == null) { SetReason("no camera"); return; }
        }

        // --- Ищем мяч по тегу ---
        if (targetBall == null && !string.IsNullOrEmpty(ballTag))
        {
            var go = GameObject.FindWithTag(ballTag);
            if (go != null) targetBall = go.transform;
        }
        if (targetBall == null) { SetReason("no target ball"); return; }

        // --- Расчёт расстояния и угла ---
        Vector3 fromCam = targetBall.position - cam.transform.position;
        float dist = fromCam.magnitude;
        dbg_distanceToBall = dist;

        if (dist > maxDistance) { SetReason($"too far ({dist:F2} > {maxDistance})"); return; }

        Vector3 flatDir = new Vector3(fromCam.x, 0f, fromCam.z);
        Vector3 flatFwd = new Vector3(cam.transform.forward.x, 0f, cam.transform.forward.z);
        if (flatDir.sqrMagnitude < 0.0001f || flatFwd.sqrMagnitude < 0.0001f)
        { SetReason("degenerate direction"); return; }

        float angle = Vector3.Angle(flatFwd, flatDir);
        dbg_angleToBall = angle;

        if (angle > hFOV * 0.5f)
        { SetReason($"out of FOV ({angle:F1}° > {hFOV * 0.5f:F1}°)"); return; }

        // --- Линия видимости: RaycastAll, пропускаем свои коллайдеры ---
        RaycastHit[] hits = Physics.RaycastAll(cam.transform.position, fromCam.normalized, dist + 0.05f);
        System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

        foreach (var hit in hits)
        {
            // Свои коллайдеры игнорируем
            if (robotRoot != null && hit.collider.transform.IsChildOf(robotRoot)) continue;

            // Если первое чужое попадание — сам мяч, всё ок
            if (hit.collider.CompareTag(ballTag)) break;

            // Иначе — стена/препятствие между камерой и мячом
            SetReason($"occluded by {hit.collider.name}");
            return;
        }

        // --- Проекция в viewport для получения нормализованного угла ---
        Vector3 vp = cam.WorldToViewportPoint(targetBall.position);
        if (vp.z <= 0f) { SetReason("behind camera"); return; }

        IsVisible = true;
        RelativeAngle = Mathf.Clamp((vp.x - 0.5f) * 2f, -1f, 1f);
        NormalizedDistance = Mathf.Clamp01(dist / maxDistance);
        dbg_lastReason = "visible";
    }

    private void SetReason(string reason)
    {
        dbg_lastReason = reason;
        if (verboseLogging && Time.time - lastLogTime > 2f)
        {
            Debug.Log($"[SimulatedYoloCamera] Ball not visible: {reason}", this);
            lastLogTime = Time.time;
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawGizmos || cam == null) return;

        Gizmos.color = IsVisible ? Color.green : new Color(1f, 1f, 1f, 0.5f);

        Vector3 origin = cam.transform.position;
        Vector3 fwd = cam.transform.forward;

        Quaternion leftRot  = Quaternion.AngleAxis(-hFOV * 0.5f, Vector3.up);
        Quaternion rightRot = Quaternion.AngleAxis(+hFOV * 0.5f, Vector3.up);

        Gizmos.DrawRay(origin, (leftRot  * fwd) * maxDistance);
        Gizmos.DrawRay(origin, (rightRot * fwd) * maxDistance);
        Gizmos.DrawRay(origin, fwd * maxDistance);

        if (IsVisible && targetBall != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(origin, targetBall.position);
        }
    }
}
