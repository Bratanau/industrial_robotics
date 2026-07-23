using UnityEngine;

/// <summary>
/// Отладочный HUD для RobotBrain: показывает все 11 наблюдений, текущие действия
/// и награду прямо поверх Game view. Полезно для проверки Heuristic-режима с WASD.
/// Вешай на любой объект в сцене (или на робота). Перетащи в поле Brain
/// ссылку на компонент RobotBrain.
/// </summary>
public class BrainDebugHUD : MonoBehaviour
{
    [SerializeField] private RobotBrain brain;

    [Header("Внешний вид")]
    [SerializeField] private int fontSize = 14;
    [SerializeField] private Vector2 position = new Vector2(20, 20);
    [SerializeField] private float panelWidth = 320f;
    [SerializeField] private Color textColor = Color.white;
    [SerializeField] private Color bgColor = new Color(0f, 0f, 0f, 0.7f);

    private GUIStyle style;
    private Texture2D bgTexture;

    private void OnGUI()
    {
        if (brain == null) return;

        if (style == null)
        {
            style = new GUIStyle(GUI.skin.label)
            {
                fontSize = fontSize,
                normal = { textColor = textColor },
                richText = true
            };
        }
        if (bgTexture == null)
        {
            bgTexture = new Texture2D(1, 1);
            bgTexture.SetPixel(0, 0, bgColor);
            bgTexture.Apply();
        }

        // ---- Собираем текст ----
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("<b>OBSERVATIONS (11)</b>");
        sb.AppendLine(Row(" 1 Ultrasonic     ", Fmt(brain.Obs01_Ultrasonic),        ColorByRange(brain.Obs01_Ultrasonic, low:true)));
        sb.AppendLine(Row(" 2 LeftIR         ", brain.Obs02_LeftIR.ToString(),      Bit(brain.Obs02_LeftIR)));
        sb.AppendLine(Row(" 3 RightIR        ", brain.Obs03_RightIR.ToString(),     Bit(brain.Obs03_RightIR)));
        sb.AppendLine(Row(" 4 GripperIR      ", brain.Obs04_GripperIR.ToString(),   Bit(brain.Obs04_GripperIR)));
        sb.AppendLine(Row(" 5 BallAngle      ", Fmt(brain.Obs05_BallAngle),         brain.Obs08_BallVisible > 0.5f ? "lime" : "gray"));
        sb.AppendLine(Row(" 6 BallDistance   ", Fmt(brain.Obs06_BallDistance),      brain.Obs08_BallVisible > 0.5f ? "lime" : "gray"));
        sb.AppendLine(Row(" 7 LastKnownAngle ", Fmt(brain.Obs07_LastKnownAngle),    "white"));
        sb.AppendLine(Row(" 8 BallVisible    ", Fmt0(brain.Obs08_BallVisible),      brain.Obs08_BallVisible > 0.5f ? "lime" : "red"));
        sb.AppendLine(Row(" 9 ServoAngle     ", Fmt(brain.Obs09_ServoAngleNorm),    "white"));
        sb.AppendLine(Row("10 HasBall        ", Fmt0(brain.Obs10_HasBall),          brain.Obs10_HasBall > 0.5f ? "lime" : "gray"));
        sb.AppendLine(Row("15 TimeSinceBall  ", Fmt(brain.Obs15_TimeSinceBallNorm), "white"));
        sb.AppendLine(Row("16 HintAngle      ", Fmt(brain.Obs16_HintAngle),      "yellow"));
        sb.AppendLine(Row("17 HintDistance   ", Fmt(brain.Obs17_HintDistance),   "yellow"));
        sb.AppendLine(Row("18 HintConf       ", Fmt(brain.Obs18_HintConfidence), "yellow"));

        sb.AppendLine();
        sb.AppendLine("<b>ACTIONS</b>");
        sb.AppendLine(Row("Gas    (W/S)     ", FmtSigned(brain.ActGas),       "cyan"));
        sb.AppendLine(Row("Steer  (A/D)     ", FmtSigned(brain.ActSteer),     "cyan"));
        sb.AppendLine(Row("Camera (Q/E)     ", FmtSigned(brain.ActCameraCmd), "cyan"));

        sb.AppendLine();
        sb.AppendLine("<b>REWARD</b>");
        sb.AppendLine(Row("Step             ", FmtSigned(brain.StepReward),
            brain.StepReward > 0 ? "lime" : (brain.StepReward < 0 ? "red" : "white")));
        sb.AppendLine(Row("Episode total    ", FmtSigned(brain.CumulativeReward),
            brain.CumulativeReward > 0 ? "lime" : (brain.CumulativeReward < 0 ? "red" : "white")));

        string text = sb.ToString();

        // ---- Отрисовка ----
        int lineCount = text.Split('\n').Length;
        float height = lineCount * (fontSize + 4) + 15;

        Rect box = new Rect(position.x, position.y, panelWidth, height);
        GUI.DrawTexture(box, bgTexture);
        GUI.Label(new Rect(box.x + 10, box.y + 5, box.width - 20, box.height - 10), text, style);
    }

    // ---- Хелперы форматирования ----
    private static string Row(string label, string value, string color)
    {
        return $"{label} <color={color}>{value}</color>";
    }

    private static string Fmt(float v)       => v.ToString("F3");
    private static string FmtSigned(float v) => v.ToString("+0.000;-0.000; 0.000");
    private static string Fmt0(float v)      => v.ToString("F0");
    private static string Bit(int v)         => v == 1 ? "red" : "lime";

    private static string ColorByRange(float v, bool low)
    {
        // low=true: маленькое значение — плохо (красный), большое — хорошо (зелёный)
        if (low)
        {
            if (v < 0.3f) return "red";
            if (v < 0.6f) return "yellow";
            return "lime";
        }
        return "white";
    }
}
