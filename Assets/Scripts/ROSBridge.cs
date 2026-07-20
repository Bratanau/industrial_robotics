using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // TwistMsg
using RosMessageTypes.Std;      // Int32Msg, Float32Msg

/// <summary>
/// Мост между обученной ML-Agents политикой (инференс) и реальным роботом GFS-X.
/// Транслирует принятые нейросетью решения в стандартные ROS-топики через
/// ROS TCP Connector, на бортовой компьютер (Raspberry Pi) робота.
///
/// EMA-сглаживание (emaAlpha) убирает высокочастотное "дрожание" сигнала между
/// соседними шагами инференса, чтобы моторы/редуктор не грелись и не изнашивались
/// от постоянных микро-рывков. При явной команде "стоп" (gas=0, steering=0)
/// сглаженные скорости сбрасываются мгновенно, без затухания — чтобы не было
/// паразитного дрейфа на "хвосте" фильтра.
/// </summary>
public class ROSBridge : MonoBehaviour
{
    [Header("Топики")]
    public string topicName = "/cmd_vel";
    public string gripperTopicName = "/cmd_gripper";
    public string cameraTopicName = "/cmd_camera_pan";

    [Header("Лимиты реального робота")]
    [Tooltip("Линейный лимит реального робота, м/с")]
    public float maxLinearSpeed = 0.5f;
    [Tooltip("Угловой лимит реального робота, рад/с")]
    public float maxAngularSpeed = 1.0f;

    [Header("Сглаживание (EMA)")]
    [Tooltip("0.8 = высокая отзывчивость, ближе к 0.1 = сильное сглаживание/инерция")]
    [Range(0.1f, 1f)]
    public float emaAlpha = 0.8f;

    [Header("Watchdog / fail-safe")]
    [Tooltip("Если за это время не поступает ни одна команда PublishCommand, мост автоматически отправит стоп.")]
    public float watchdogTimeout = 0.5f;

    private ROSConnection ros;
    private float smoothGas;
    private float smoothSteering;
    private float lastCommandTime = -1f;
    private bool watchdogArmed;
    private bool watchdogWarned;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<TwistMsg>(topicName);
        ros.RegisterPublisher<Int32Msg>(gripperTopicName);
        ros.RegisterPublisher<Float32Msg>(cameraTopicName);

        lastCommandTime = Time.time;
        watchdogArmed = false;
        watchdogWarned = false;
    }

    private void Update()
    {
        if (!watchdogArmed || ros == null) return;

        if (Time.time - lastCommandTime > watchdogTimeout)
        {
            if (!watchdogWarned)
            {
                Debug.LogWarning("[ROSBridge] Watchdog: не получено ни одной команды PublishCommand за 0.5s. Отправляю аварийный STOP.");
                watchdogWarned = true;
            }

            PublishCommandInternal(0f, 0f, true);
        }
    }

    /// <summary>
    /// Сглаживает и публикует линейную/угловую скорость в /cmd_vel.
    /// Вызывать каждый шаг инференса из RobotBrain.OnActionReceived().
    /// </summary>
    public void PublishCommand(float gas, float steering)
    {
        watchdogArmed = true;
        lastCommandTime = Time.time;
        watchdogWarned = false;
        PublishCommandInternal(gas, steering, false);
    }

    private void PublishCommandInternal(float gas, float steering, bool emergencyStop)
    {
        if (emergencyStop || (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steering, 0f)))
        {
            // Hard stop: сбрасываем сглаженные значения мгновенно, без затухания,
            // чтобы избежать паразитного дрейфа от "хвоста" EMA-фильтра.
            smoothGas = 0f;
            smoothSteering = 0f;
        }
        else
        {
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteering = emaAlpha * steering + (1f - emaAlpha) * smoothSteering;
        }

        TwistMsg cmd = new TwistMsg();
        cmd.linear.x = smoothGas * maxLinearSpeed;
        cmd.angular.z = smoothSteering * maxAngularSpeed;

        if (ros != null)
            ros.Publish(topicName, cmd);
    }

    /// <summary>
    /// Публикует команду клешни в /cmd_gripper.
    /// Ожидаемые значения совпадают с дискретным действием агента: 0 = ничего, 1 = закрыть, 2 = открыть.
    /// </summary>
    public void PublishGripperCmd(int cmd)
    {
        Int32Msg msg = new Int32Msg();
        msg.data = cmd;
        if (ros != null)
            ros.Publish(gripperTopicName, msg);
    }

    /// <summary>
    /// Публикует угол поворота сервопривода камеры в /cmd_camera_pan.
    /// </summary>
    public void PublishCameraCmd(float yaw)
    {
        Float32Msg msg = new Float32Msg();
        msg.data = yaw;
        if (ros != null)
            ros.Publish(cameraTopicName, msg);
    }
}