using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Collections.Concurrent;

/// <summary>
/// Practice 7: HIL vision receiver. Listens on UDP :5005 for JSON packets from
/// yolo_vision_node.py (real YOLO on the PC), converts them to the SAME
/// convention the simulator camera uses, and exposes the same API
/// (IsVisible / RelativeAngle / NormalizedDistance) so RobotBrain can switch
/// between simulated and real vision without retraining.
///
/// IMPORTANT convention fix vs the handout:
///   Python sends distance = bboxHeight/frameHeight  (1.0 = CLOSE, 0 = far)
///   The trained model expects NormalizedDistance     (0 = CLOSE, 1.0 = far)
/// This script inverts and calibrates the value. Without this inversion the
/// policy would read "far" when the ball is close and vice versa.
/// </summary>
public class RealVision : MonoBehaviour
{
    [Header("Network")]
    public int udpPort = 5005;
    [Tooltip("Master switch: when ON, RobotBrain reads vision from here instead of the simulated camera")]
    public bool useYOLO = false;
    [Tooltip("If no packet arrives for this many seconds, the ball is considered lost (node crashed / WiFi drop)")]
    public float signalTimeout = 0.5f;

    [Header("Distance calibration (bbox height fraction -> sim distance)")]
    [Tooltip("bbox height fraction when the ball is right at the gripper (measure it live!). Maps to NormalizedDistance = 0")]
    public float bboxAtGripper = 0.45f;
    [Tooltip("bbox height fraction at the edge of usable range (~2 m). Maps to NormalizedDistance = 1")]
    public float bboxAtMaxRange = 0.04f;

    [Header("Telemetry (read-only, same API as SimulatedYoloCamera)")]
    [SerializeField] private bool seesBall;
    [SerializeField] private float relativeAngle;      // -1..1, 0 = centered
    [SerializeField] private float normalizedDistance; // 0 = close, 1 = far (SIM convention)
    [SerializeField] private float confidence;
    [SerializeField] private float lastPacketAge;

    public bool  IsVisible          => useYOLO && seesBall;
    public float RelativeAngle      => relativeAngle;
    public float NormalizedDistance => normalizedDistance;
    public float Confidence         => confidence;

    [System.Serializable]
    public class YoloDataPacket
    {
        public float angle;      // -1 left .. +1 right
        public float distance;   // bbox height / frame height (1 = close!)
        public float sees;       // 1 = visible
        public float conf;
        public float w;
        public float h;
    }

    private CancellationTokenSource cts;
    private readonly ConcurrentQueue<YoloDataPacket> udpQueue = new ConcurrentQueue<YoloDataPacket>();
    private float lastPacketTime = -999f;

    void Start()
    {
        cts = new CancellationTokenSource();
        Task.Run(() => UdpListenerLoop(cts.Token));
    }

    private async Task UdpListenerLoop(CancellationToken token)
    {
        try
        {
            using (var udpClient = new UdpClient(udpPort))
            {
                while (!token.IsCancellationRequested)
                {
                    var result = await udpClient.ReceiveAsync();
                    string json = System.Text.Encoding.UTF8.GetString(result.Buffer);
                    var packet = JsonUtility.FromJson<YoloDataPacket>(json);
                    if (packet != null) udpQueue.Enqueue(packet);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[RealVision] UDP listener stopped: {e.Message}");
        }
    }

    void Update()
    {
        // Drain the queue on the main thread; keep only the freshest packet
        while (udpQueue.TryDequeue(out var packet))
        {
            lastPacketTime = Time.time;
            seesBall   = packet.sees > 0.5f;
            confidence = packet.conf;

            if (seesBall)
            {
                relativeAngle = Mathf.Clamp(packet.angle, -1f, 1f);

                // --- INVERSION + calibration to the simulator convention ---
                // packet.distance: bboxAtMaxRange (far) .. bboxAtGripper (close)
                // normalizedDistance: 1 (far) .. 0 (close)
                float t = Mathf.InverseLerp(bboxAtMaxRange, bboxAtGripper, packet.distance);
                normalizedDistance = Mathf.Clamp01(1f - t);
            }
            else
            {
                relativeAngle = 0f;
                normalizedDistance = 1f; // far / not seen (matches sim behavior)
            }
        }

        // Watchdog: node crashed or WiFi dropped -> declare the ball lost
        lastPacketAge = Time.time - lastPacketTime;
        if (seesBall && lastPacketAge > signalTimeout)
        {
            seesBall = false;
            relativeAngle = 0f;
            normalizedDistance = 1f;
        }
    }

    void OnDestroy()
    {
        cts?.Cancel();
    }
}
