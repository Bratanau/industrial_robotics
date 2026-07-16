using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Competition-layout obstacle randomizer. On every episode reset it scatters
/// the arena obstacles inside a middle belt between the robot's start side and
/// the ball zone, with random yaw, avoiding overlaps.
///
/// This is TASK geometry (competition rules), not Sim2Real noise, so it lives
/// in its own component and is ALWAYS active - independent of DomainRandomizer.
///
/// Setup: add to the TrainingArea prefab root (or the robot), drag the obstacle
/// transforms into the list, assign arenaCenter (Floor) and robotSpawn
/// (the robot itself works: its Initialize pose is the spawn).
/// RobotBrain calls Shuffle() from OnEpisodeBegin BEFORE spawning the ball.
/// </summary>
public class ObstacleRandomizer : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Obstacles to scatter (the cubes). Their Y position is preserved")]
    [SerializeField] private List<Transform> obstacles = new List<Transform>();
    [Tooltip("Arena floor / center reference")]
    [SerializeField] private Transform arenaCenter;
    [Tooltip("Robot transform (start pose defines the belt orientation)")]
    [SerializeField] private Transform robot;

    [Header("Belt geometry (relative to the robot start, along its forward)")]
    [Tooltip("Belt near edge: meters ahead of the robot start")]
    [SerializeField] private float beltMinForward = 0.8f;
    [Tooltip("Belt far edge: meters ahead of the robot start")]
    [SerializeField] private float beltMaxForward = 2.0f;
    [Tooltip("Half-width of the belt sideways")]
    [SerializeField] private float beltHalfWidth = 1.6f;

    [Header("Placement rules")]
    [Tooltip("Minimum center-to-center distance between obstacles")]
    [SerializeField] private float minSpacing = 0.55f;
    [Tooltip("Keep obstacles at least this far from arena walls")]
    [SerializeField] private float wallMargin = 0.3f;
    [Tooltip("Random yaw for each obstacle")]
    [SerializeField] private bool randomYaw = true;
    [SerializeField] private float arenaHalf = 2.0f;

    private Vector3 startPos;
    private Quaternion startRot;
    private bool captured;

    private void Awake()
    {
        CaptureStart();
    }

    private void CaptureStart()
    {
        if (captured || robot == null) return;
        startPos = robot.position;
        startRot = robot.rotation;
        captured = true;
    }

    /// <summary>Scatter obstacles. Call from RobotBrain.OnEpisodeBegin BEFORE ball spawn.</summary>
    public void Shuffle()
    {
        CaptureStart();
        if (obstacles.Count == 0) return;

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
                p.y = obs.position.y;                       // keep original height
                p.x = Mathf.Clamp(p.x, c.x - lim, c.x + lim);
                p.z = Mathf.Clamp(p.z, c.z - lim, c.z + lim);

                ok = true;
                foreach (var q in placed)
                    if (Vector2.Distance(new Vector2(p.x, p.z), new Vector2(q.x, q.z)) < minSpacing)
                    { ok = false; break; }
            }
            // even if not ok after 40 tries, place at the last candidate:
            // a rare overlap is better than an obstacle stuck at its old spot
            obs.position = p;
            if (randomYaw)
                obs.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
            placed.Add(p);
        }
    }
}
