using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// AI Command UI — send natural language commands to the robot via Gemini.
///
/// Setup in Unity:
///   1. Create a Panel on Canvas (left or bottom area)
///   2. Add:
///        - TMP_InputField  (assign to CommandInput)
///        - Button          (assign to SendButton)
///        - TMP_Text        (assign to ReplyText)
///   3. Attach this script to any GameObject (e.g. ROSManager)
/// </summary>
public class AICommandUI : MonoBehaviour
{
    [Header("UI References")]
    public TMP_InputField CommandInput;   // user types here
    public Button SendButton;    // click to send
    public TMP_Text ReplyText;     // shows AI reply

    [Header("Settings")]
    public string PlaceholderText = "e.g. go to phone with speed 30%";
    public float ReplyDisplaySecs = 5f;   // how long reply stays visible

    // ── ROS ──────────────────────────────────────────────────────────────
    private const string AI_COMMAND_TOPIC = "/ai_command";
    private const string AI_REPLY_TOPIC = "/ai_reply";

    private ROSConnection _ros;
    private bool _waiting = false;

    // ── Unity lifecycle ───────────────────────────────────────────────────

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<StringMsg>(AI_COMMAND_TOPIC);
        _ros.Subscribe<StringMsg>(AI_REPLY_TOPIC, OnAIReply);

        // Wire up button
        if (SendButton != null)
            SendButton.onClick.AddListener(OnSendClicked);

        // Submit on Enter key in input field
        if (CommandInput != null)
            CommandInput.onSubmit.AddListener(_ => OnSendClicked());

        SetReply("");
        SetSendingState(false);

        Debug.Log("[AICommandUI] Ready — publish to /ai_command");
    }

    // ── Send ──────────────────────────────────────────────────────────────

    void OnSendClicked()
    {
        if (CommandInput == null) return;

        string text = CommandInput.text.Trim();
        if (string.IsNullOrEmpty(text)) return;
        if (_waiting) return;

        // Publish to /ai_command
        _ros.Publish(AI_COMMAND_TOPIC, new StringMsg(text));
        Debug.Log($"[AICommandUI] Sent: '{text}'");

        // UI feedback
        SetReply("Thinking...");
        SetSendingState(true);

        // Clear input
        CommandInput.text = "";
        CommandInput.ActivateInputField();
    }

    // ── Receive AI reply ──────────────────────────────────────────────────

    void OnAIReply(StringMsg msg)
    {
        string reply = msg.data;
        Debug.Log($"[AICommandUI] Reply: '{reply}'");

        // Must update UI on main thread
        UnityMainThreadDispatcher.Enqueue(() =>
        {
            SetReply(reply);
            SetSendingState(false);
            StartCoroutine(ClearReplyAfter(ReplyDisplaySecs));
        });
    }

    // ── UI helpers ────────────────────────────────────────────────────────

    void SetSendingState(bool sending)
    {
        _waiting = sending;
        if (SendButton != null)
        {
            SendButton.interactable = !sending;
            var btnText = SendButton.GetComponentInChildren<TMP_Text>();
            if (btnText != null)
                btnText.text = sending ? "..." : "Send";
        }
    }

    void SetReply(string text)
    {
        if (ReplyText != null)
            ReplyText.text = text;
    }

    IEnumerator ClearReplyAfter(float seconds)
    {
        yield return new WaitForSeconds(seconds);
        SetReply("");
    }
}

/// <summary>
/// Simple main-thread dispatcher for ROS callbacks.
/// Attach to any persistent GameObject (e.g. ROSManager).
/// </summary>
public class UnityMainThreadDispatcher : MonoBehaviour
{
    private static readonly System.Collections.Generic.Queue<System.Action>
        _queue = new();
    private static UnityMainThreadDispatcher _instance;

    public static void Enqueue(System.Action action)
    {
        if (_instance == null)
        {
            var go = new GameObject("MainThreadDispatcher");
            _instance = go.AddComponent<UnityMainThreadDispatcher>();
            DontDestroyOnLoad(go);
        }
        lock (_queue) { _queue.Enqueue(action); }
    }

    void Update()
    {
        lock (_queue)
        {
            while (_queue.Count > 0)
                _queue.Dequeue()?.Invoke();
        }
    }
}