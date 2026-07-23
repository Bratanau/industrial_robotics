using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents; // <-- IMPORTANTE: Agregar este namespace

public class ObstacleRandomizer : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private List<Transform> obstacles = new List<Transform>();
    [SerializeField] private Transform arenaCenter;
    [SerializeField] private Transform robot;

    [Header("Belt geometry (relative to the robot start, along its forward)")]
    [SerializeField] private float beltMinForward = 0.8f;
    [SerializeField] private float beltMaxForward = 2.0f;
    [SerializeField] private float beltHalfWidth = 1.6f;

    [Header("Placement rules")]
    [SerializeField] private float minSpacing = 0.55f;
    [SerializeField] private float wallMargin = 0.3f;
    [SerializeField] private bool randomYaw = true;
    [SerializeField] private float arenaHalf = 2.0f;

    // Posiciones fijas guardadas al inicio para el modo obstacle_mode = 0
    private List<Vector3> initialObstaclePositions = new List<Vector3>();
    private List<Quaternion> initialObstacleRotations = new List<Quaternion>();

    private Vector3 startPos;
    private Quaternion startRot;
    private bool captured;

    private void Awake()
    {
        CaptureStart();
        
        // Guardar la configuración por defecto/fija de los obstáculos
        foreach (var obs in obstacles)
        {
            if (obs != null)
            {
                initialObstaclePositions.Add(obs.position);
                initialObstacleRotations.Add(obs.rotation);
            }
        }
    }

    private void CaptureStart()
    {
        if (captured || robot == null) return;
        startPos = robot.position;
        startRot = robot.rotation;
        captured = true;
    }

    /// <summary>Scatter obstacles according to YAML Curriculum mode.</summary>
    public void Shuffle()
    {
        CaptureStart();
        if (obstacles.Count == 0) return;

        // --- LECTURA DEL PARAMETRO DE CURRICULUM ---
        // 0 = Fijo (restaurar original), 1 = Aleatorio (dispersar)
        float mode = Academy.Instance.EnvironmentParameters.GetWithDefault("obstacle_mode", 1f);

        if (mode < 0.5f)
        {
            // MODO FIJO (Fases 0, 2, 4): Restaurar a posiciones originales
            for (int i = 0; i < obstacles.Count; i++)
            {
                if (obstacles[i] != null && i < initialObstaclePositions.Count)
                {
                    obstacles[i].position = initialObstaclePositions[i];
                    obstacles[i].rotation = initialObstacleRotations[i];
                }
            }
            return;
        }

        // MODO RANDOM (Fases 1, 3, 5, 6): Dispersar en la franja/belt
        Vector3 fwd  = startRot * Vector3.forward;
        Vector3 side = startRot * Vector3.right;
        Vector3 c = arenaCenter != null ? arenaCenter.position : Vector3.zero;
        float lim = arenaHalf - wallMargin;

        var placed = new List<Vector3>();
        foreach (var obs in obstacles)
        {
            if (obs == null) continue;
            Vector3 p = obs.position;
            bool ok = false;
            for (int guard = 0; guard < 40 && !ok; guard++)
            {
                p = startPos
                  + fwd  * Random.Range(beltMinForward, beltMaxForward)
                  + side * Random.Range(-beltHalfWidth, beltHalfWidth);
                p.y = obs.position.y;
                p.x = Mathf.Clamp(p.x, c.x - lim, c.x + lim);
                p.z = Mathf.Clamp(p.z, c.z - lim, c.z + lim);

                ok = true;
                foreach (var q in placed)
                    if (Vector2.Distance(new Vector2(p.x, p.z), new Vector2(q.x, q.z)) < minSpacing)
                    { ok = false; break; }
            }

            obs.position = p;
            if (randomYaw)
                obs.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
            placed.Add(p);
        }
    }
}
