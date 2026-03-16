using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// FR5 Robot Keyboard Controller — Unity New Input System
///
/// JOINT MODE (default)
///   Q / A  →  Joint 1  +/-
///   W / S  →  Joint 2  +/-
///   E / D  →  Joint 3  +/-
///   R / F  →  Joint 4  +/-
///   T / G  →  Joint 5  +/-
///   Y / H  →  Joint 6  +/-
///
/// COMMON
///   M       →  cycle mode (Joint → Base Cart → Tool Cart)
///   Space   →  stop jog
///   Escape  →  emergency stop
///   [ / ]   →  speed -5% / +5%
/// </summary>
public class FR5KeyboardController : MonoBehaviour
{
    [Header("ROS Topics")]
    public string jogTopic = "/fr_jog_cmd";
    public string cmdTopic = "/fr_robot_cmd";
    public string stateTopic = "/nonrt_state_data";

    [Header("Jog Settings")]
    [Range(1f, 100f)] public float jogVelocity = 10f;
    public float maxDis = 30f;
    public float speedStep = 5f;

    // ── Internal ──────────────────────────────────────────────────
    ROSConnection ros;
    int jogMode = 0;
    int activeAxis = -1;
    bool isJogging = false;

    // Robot state
    bool emergencyStop = false;
    bool collisionState = false;
    int robotState = 0;
    float[] jtCurPos = new float[6];

    static readonly string[] ModeNames = { "Joint", "Base Cart", "Tool Cart" };

    // (positive key, negative key, axis 1-based)
    static readonly (Key pos, Key neg, int axis)[] KeyMap =
    {
        (Key.Q, Key.A, 1),
        (Key.W, Key.S, 2),
        (Key.E, Key.D, 3),
        (Key.R, Key.F, 4),
        (Key.T, Key.G, 5),
        (Key.Y, Key.H, 6),
    };

    // ── Unity lifecycle ───────────────────────────────────────────
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(jogTopic);
        ros.RegisterPublisher<StringMsg>(cmdTopic);
        ros.Subscribe<StringMsg>(stateTopic, OnStateData);

        Debug.Log("[FR5] Keyboard controller ready (New Input System)");
    }

    void Update()
    {
        var kb = Keyboard.current;
        if (kb == null) return;

        HandleSpeedKeys(kb);
        HandleModeSwitch(kb);
        HandleEmergencyStop(kb);
        HandleJogKeys(kb);
    }

    // ── Input handlers ────────────────────────────────────────────
    void HandleSpeedKeys(Keyboard kb)
    {
        if (kb[Key.LeftBracket].wasPressedThisFrame)
        {
            jogVelocity = Mathf.Max(1f, jogVelocity - speedStep);
            Debug.Log($"[FR5] Speed: {jogVelocity}%");
        }
        if (kb[Key.RightBracket].wasPressedThisFrame)
        {
            jogVelocity = Mathf.Min(100f, jogVelocity + speedStep);
            Debug.Log($"[FR5] Speed: {jogVelocity}%");
        }
    }

    void HandleModeSwitch(Keyboard kb)
    {
        if (kb[Key.M].wasPressedThisFrame)
        {
            if (isJogging) SendJog(activeAxis, 0);
            jogMode = (jogMode + 1) % 3;
            isJogging = false;
            activeAxis = -1;
            Debug.Log($"[FR5] Mode → {ModeNames[jogMode]}");
        }
    }

    void HandleEmergencyStop(Keyboard kb)
    {
        if (kb[Key.Escape].wasPressedThisFrame)
        {
            Debug.LogWarning("[FR5] ⚠ EMERGENCY STOP");
            SendJog(activeAxis, 0);
            SendCmd("ImmStopJOG()");
            isJogging = false;
            activeAxis = -1;
        }
    }

    void HandleJogKeys(Keyboard kb)
    {
        // Space = stop
        if (kb[Key.Space].wasPressedThisFrame)
        {
            SendJog(activeAxis, 0);
            isJogging = false;
            activeAxis = -1;
            return;
        }

        foreach (var (posKey, negKey, axis) in KeyMap)
        {
            bool posPressed = kb[posKey].wasPressedThisFrame;
            bool negPressed = kb[negKey].wasPressedThisFrame;
            bool posReleased = kb[posKey].wasReleasedThisFrame;
            bool negReleased = kb[negKey].wasReleasedThisFrame;
            bool posHeld = kb[posKey].isPressed;
            bool negHeld = kb[negKey].isPressed;

            if (posPressed)
            {
                if (isJogging && activeAxis != axis) SendJog(activeAxis, 0);
                SendJog(axis, 1);
                activeAxis = axis;
                isJogging = true;
            }
            else if (negPressed)
            {
                if (isJogging && activeAxis != axis) SendJog(activeAxis, 0);
                SendJog(axis, -1);
                activeAxis = axis;
                isJogging = true;
            }

            if (activeAxis == axis && (posReleased || negReleased))
            {
                if (!posHeld && !negHeld)
                {
                    SendJog(axis, 0);
                    isJogging = false;
                    activeAxis = -1;
                }
            }
        }
    }

    // ── ROS publishers ────────────────────────────────────────────
    void SendJog(int axis, int direction)
    {
        if (axis < 1 && direction == 0) return;

        string json = $"{{\"mode\":{jogMode},\"axis\":{axis}," +
                      $"\"direction\":{direction},\"velocity\":{jogVelocity}," +
                      $"\"max_dis\":{maxDis}}}";
        ros.Publish(jogTopic, new StringMsg { data = json });
    }

    void SendCmd(string cmd)
    {
        ros.Publish(cmdTopic, new StringMsg { data = cmd });
    }

    // ── State subscriber ──────────────────────────────────────────
    void OnStateData(StringMsg msg)
    {
        try
        {
            var state = JsonUtility.FromJson<RobotStateJson>(msg.data);
            emergencyStop = state.emergency_stop;
            collisionState = state.collision_state;
            robotState = state.robot_state;

            if (state.jt_cur_pos != null && state.jt_cur_pos.Length == 6)
                jtCurPos = state.jt_cur_pos;

            if ((emergencyStop || collisionState) && isJogging)
            {
                Debug.LogWarning("[FR5] Auto-stop: emergency or collision");
                isJogging = false;
                activeAxis = -1;
            }
        }
        catch { }
    }

    // ── Public helpers for UI buttons ─────────────────────────────
    //public void EnableRobot() => SendCmd("RobotEnable(1)");
    //public void DisableRobot() => SendCmd("RobotEnable(0)");
    //public void SetAutoMode() => SendCmd("Mode(0)");
    //public void SetManualMode() => SendCmd("Mode(1)");
    //public void ResetErrors() => SendCmd("ResetAllError()");
    //public void StopMotion() => SendCmd("ImmStopJOG()");

    // ── HUD ───────────────────────────────────────────────────────
    void OnGUI()
    {
        float w = 320f, h = 240f;
        GUI.Box(new Rect(10, 10, w, h), "FR5 Robot Controller");

        string[] stateNames = { "?", "Stop", "Running", "Paused", "Drag" };
        string stateName = (robotState >= 0 && robotState < stateNames.Length)
            ? stateNames[robotState] : robotState.ToString();

        float y = 35f;
        GUI.Label(new Rect(20, y, w - 20, 20),
            $"State: {stateName}  Mode: {ModeNames[jogMode]}  Speed: {jogVelocity}%");
        y += 20;
        GUI.Label(new Rect(20, y, w - 20, 20),
            $"J: {jtCurPos[0]:F1} {jtCurPos[1]:F1} {jtCurPos[2]:F1} " +
            $"{jtCurPos[3]:F1} {jtCurPos[4]:F1} {jtCurPos[5]:F1}");
        y += 20;

        GUI.contentColor = (emergencyStop || collisionState) ? Color.red : Color.green;
        GUI.Label(new Rect(20, y, w - 20, 20),
            emergencyStop ? "⚠ EMERGENCY STOP" : collisionState ? "⚠ COLLISION" : "● OK");
        GUI.contentColor = Color.white;
        y += 25;

        GUI.Label(new Rect(20, y, w - 20, 20), "Q/A=J1  W/S=J2  E/D=J3"); y += 18;
        GUI.Label(new Rect(20, y, w - 20, 20), "R/F=J4  T/G=J5  Y/H=J6"); y += 18;
        GUI.Label(new Rect(20, y, w - 20, 20), "M=mode  SPC=stop  ESC=e-stop  [/]=spd"); y += 25;

        //if (GUI.Button(new Rect(20, y, 65, 22), "Enable")) EnableRobot();
        //if (GUI.Button(new Rect(90, y, 65, 22), "Disable")) DisableRobot();
        //if (GUI.Button(new Rect(160, y, 65, 22), "Manual")) SetManualMode();
        //if (GUI.Button(new Rect(230, y, 65, 22), "Auto")) SetAutoMode();
        //y += 28;
        //if (GUI.Button(new Rect(20, y, 65, 22), "Rst Err")) ResetErrors();
        //if (GUI.Button(new Rect(90, y, 65, 22), "Stop")) StopMotion();
    }

    // ── JSON class ────────────────────────────────────────────────
    [System.Serializable]
    class RobotStateJson
    {
        public int program_state;
        public int robot_state;
        public int robot_mode;
        public float[] jt_cur_pos;
        public float[] tl_cur_pos;
        public bool emergency_stop;
        public bool collision_state;
        public bool motion_done;
    }
}