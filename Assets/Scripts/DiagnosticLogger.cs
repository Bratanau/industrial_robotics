using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization; // Обязательно для InvariantCulture!

/// <summary>
/// Записывает диагностический CSV-лог заезда робота — зрение, физические сенсоры,
/// действия ИИ и одометрию на каждом шаге принятия решения. Используется для
/// разбора расхождений между поведением в симуляции и на реальном роботе (Sim-to-Real).
/// </summary>
public class DiagnosticLogger : MonoBehaviour
{
    public bool enableLogging = false;  // Флаг включения записи логов
    public int logEveryN = 1;           // Записывать каждый N-й шаг (1 = каждый)
    public int maxRows = 2000;          // Ограничение размера файла (строк)

    private StreamWriter writer;
    private int rowsWritten = 0;
    private float startTime;
    private int stepCounter = 0;

    private void Start()
    {
        if (enableLogging)
        {
            // Путь к файлу: на один уровень выше папки Assets.
            string path = Path.Combine(Application.dataPath, "..", "diagnostic_log.csv");

            // Открываем StreamWriter (false означает перезаписывать файл при каждом новом запуске).
            writer = new StreamWriter(path, false, Encoding.UTF8);

            // Записываем заголовок колонок CSV (строго в одну строчку без пробелов).
            writer.WriteLine("time,step,ballSeen,ballAngle,ballDist,uz,irL,irR,gripIR,camYaw,gas,steering,hasBall,holdTicks,isRetrying,displacementX,displacementZ,heading,speed");

            startTime = Time.time;
            Debug.Log($"[DiagnosticLogger] Запись лога запущена в: {path}");
        }
    }

    /// <summary>
    /// Записывает одну строку телеметрии. Вызывать раз за шаг принятия решения
    /// (например, из конца RobotBrain.OnActionReceived).
    /// </summary>
    public void LogStep(
        int step, bool ballSeen, float ballAngle, float ballDist,
        float uz, int irL, int irR, int gripIR, float camYaw,
        float gas, float steering, bool hasBall, int holdTicks,
        bool isRetrying, float displacementX, float displacementZ,
        float heading, float speed)
    {
        if (!enableLogging || writer == null || rowsWritten >= maxRows) return;

        // Прореживание — пишем только каждый logEveryN-й вызов.
        stepCounter++;
        if (logEveryN > 1 && stepCounter % logEveryN != 0) return;

        float elapsed = Time.time - startTime;

        // Сборка строки с принудительным использованием CultureInfo.InvariantCulture,
        // чтобы десятичный разделитель всегда был точкой, независимо от локали ОС.
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3},{1},{2},{3:F4},{4:F4},{5:F4},{6},{7},{8},{9:F4},{10:F4},{11:F4},{12},{13},{14},{15:F4},{16:F4},{17:F4},{18:F4}",
            elapsed, step, ballSeen ? 1 : 0, ballAngle, ballDist, uz, irL, irR, gripIR, camYaw,
            gas, steering, hasBall ? 1 : 0, holdTicks, isRetrying ? 1 : 0,
            displacementX, displacementZ, heading, speed);

        writer.WriteLine(line);
        writer.Flush(); // Сбрасываем буфер в файл, чтобы данные не пропали при сбоях.
        rowsWritten++;

        if (rowsWritten >= maxRows)
        {
            Debug.Log($"[DiagnosticLogger] Сбор лога завершен. Достигнут лимит {maxRows} строк.");
        }
    }

    // Закрываем файл при уничтожении объекта (например, при выходе из игры).
    private void OnDestroy()
    {
        writer?.Close();
    }
}