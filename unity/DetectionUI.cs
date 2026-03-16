using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// Builds a persistent UI panel with one button per target object.
/// Also spawns primitive 3D objects (car, cell phone) in the scene
/// that move in real time to detected robot XY positions.
///
/// Setup in Unity:
///   1. Create Canvas → Panel (assign to RootPanel)
///   2. Add Vertical Layout Group to Panel
///   3. Create Button prefab with TMP_Text child named "Label"
///   4. Attach this script to any GameObject
///   5. Assign DetectionSubscriber, RootPanel, ButtonPrefab, StatusText
///   6. Set TableY to your table surface Y in Unity scene
/// </summary>
public class DetectionUI : MonoBehaviour
{
    // ── Target labels ─────────────────────────────────────────────────────
    private static readonly List<string> TARGET_LABELS = new() { "car", "cell phone" };

    // ── UI References ─────────────────────────────────────────────────────
    [Header("UI References")]
    public DetectionSubscriber Subscriber;
    public Transform RootPanel;
    public GameObject ButtonPrefab;

    [Header("Status")]
    public TMP_Text StatusText;

    [Header("UI Settings")]
    [Tooltip("Seconds between UI refreshes")]
    public float RefreshInterval = 0.2f;

    // ── 3D Scene Settings ─────────────────────────────────────────────────
    [Header("3D Object Settings")]
    public Transform RobotBase;   // ← drag robot root object here

    [Tooltip("Y position of table surface in Unity scene")]
    public float TableY = 0f;

    [Tooltip("Robot mm to Unity units. 0.001 = 1mm becomes 0.001 Unity units")]
    public float ScaleFactor = 0.001f;

    [Tooltip("How fast 3D objects slide to new position")]
    public float MoveSpeed = 5f;

    // ── Internal UI state ─────────────────────────────────────────────────
    private float _timer = 0f;
    private Dictionary<string, GameObject> _buttonMap = new();
    private List<DetectionSubscriber.Detection> _lastDetections = new();

    // ── Internal 3D state ─────────────────────────────────────────────────
    private Dictionary<string, GameObject> _sceneObjects = new();
    private Dictionary<string, Vector3> _targetPos = new();

    // ═════════════════════════════════════════════════════════════════════
    // Unity lifecycle
    // ═════════════════════════════════════════════════════════════════════

    void Start()
    {
        if (Subscriber == null)
            Subscriber = FindObjectOfType<DetectionSubscriber>();

        if (Subscriber == null) Debug.LogError("[DetectionUI] DetectionSubscriber not found!");
        if (ButtonPrefab == null) Debug.LogError("[DetectionUI] ButtonPrefab not assigned!");
        if (RootPanel == null) Debug.LogError("[DetectionUI] RootPanel not assigned!");

        // Create UI buttons
        foreach (var label in TARGET_LABELS)
            CreateButton(label);

        // Create 3D scene objects
        foreach (var label in TARGET_LABELS)
            SpawnSceneObject(label);

        SetStatus("Waiting for detections...");
    }

    void Update()
    {
        // UI + 3D refresh
        _timer += Time.deltaTime;
        if (_timer >= RefreshInterval)
        {
            _timer = 0f;
            if (Subscriber != null && Subscriber.DetectionsUpdated)
            {
                Subscriber.DetectionsUpdated = false;
                var detections = Subscriber.GetDetections();
                UpdateButtons(detections);
                UpdateSceneObjects(detections);
            }
        }

        // Smooth 3D movement
        foreach (var kvp in _sceneObjects)
        {
            if (!_targetPos.ContainsKey(kvp.Key)) continue;
            kvp.Value.transform.position = Vector3.Lerp(
                kvp.Value.transform.position,
                _targetPos[kvp.Key],
                Time.deltaTime * MoveSpeed);
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    // UI Buttons
    // ═════════════════════════════════════════════════════════════════════

    void CreateButton(string label)
    {
        if (ButtonPrefab == null || RootPanel == null) return;

        var btnObj = Instantiate(ButtonPrefab, RootPanel);
        _buttonMap[label] = btnObj;

        var tmp = btnObj.GetComponentInChildren<TMP_Text>();
        if (tmp != null)
            tmp.text = $"{label}\n(not detected)";

        var image = btnObj.GetComponent<Image>();
        if (image != null)
            image.color = new Color(0.75f, 0.75f, 0.75f, 0.5f);

        var button = btnObj.GetComponent<Button>();
        if (button != null)
        {
            button.interactable = false;
            string capturedLabel = label;
            button.onClick.AddListener(() => OnButtonClicked(capturedLabel));
        }
    }

    void UpdateButtons(List<DetectionSubscriber.Detection> detections)
    {
        var detMap = new Dictionary<string, DetectionSubscriber.Detection>();
        if (detections != null)
            foreach (var det in detections)
                if (TARGET_LABELS.Contains(det.label))
                    detMap[det.label] = det;

        int activeCount = 0;

        foreach (var label in TARGET_LABELS)
        {
            if (!_buttonMap.ContainsKey(label)) continue;

            var btnObj = _buttonMap[label];
            var button = btnObj.GetComponent<Button>();
            var image = btnObj.GetComponent<Image>();
            var tmp = btnObj.GetComponentInChildren<TMP_Text>();

            if (detMap.ContainsKey(label))
            {
                var det = detMap[label];
                string robotXY = det.robot_xy_mm != null && det.robot_xy_mm.Length >= 2
                    ? $"X={det.robot_xy_mm[0]:F1}  Y={det.robot_xy_mm[1]:F1} mm"
                    : "N/A";

                if (tmp != null) tmp.text = $"{det.label}   {det.confidence:P0}\nRobot: {robotXY}";
                if (button != null) button.interactable = true;
                if (image != null) image.color = GetColorForLabel(label);
                activeCount++;
            }
            else
            {
                if (tmp != null) tmp.text = $"{label}\n(not detected)";
                if (button != null) button.interactable = false;
                if (image != null) image.color = new Color(0.75f, 0.75f, 0.75f, 0.5f);
            }
        }

        _lastDetections = detections ?? new List<DetectionSubscriber.Detection>();
        SetStatus(activeCount > 0
            ? $"{activeCount} object(s) detected"
            : "No target objects detected");
    }

    void OnButtonClicked(string label)
    {
        if (Subscriber == null) return;
        Subscriber.SendCommand(label);
        Debug.Log($"[DetectionUI] Command sent: '{label}'");
        StartCoroutine(FlashStatus($"Moving to: {label}"));
    }

    // ═════════════════════════════════════════════════════════════════════
    // 3D Scene Objects
    // ═════════════════════════════════════════════════════════════════════

    void SpawnSceneObject(string label)
    {
        var root = new GameObject($"[{label}]");
        root.transform.SetParent(transform);
        root.SetActive(false);
        _sceneObjects[label] = root;

        switch (label)
        {
            case "car": BuildCar(root); break;
            case "cell phone": BuildPhone(root); break;
        }

        AddFloatingLabel(root, label);
    }

    void UpdateSceneObjects(List<DetectionSubscriber.Detection> detections)
    {
        var detMap = new Dictionary<string, DetectionSubscriber.Detection>();
        if (detections != null)
            foreach (var det in detections)
                detMap[det.label] = det;

        foreach (var label in TARGET_LABELS)
        {
            if (!_sceneObjects.ContainsKey(label)) continue;
            var obj = _sceneObjects[label];

            if (detMap.ContainsKey(label))
            {
                var det = detMap[label];
                if (det.robot_xy_mm == null || det.robot_xy_mm.Length < 2) continue;

                // Robot X → Unity X
                // Robot Y → Unity Z  (robot Y = depth = Unity forward)
                float ux = -det.robot_xy_mm[0] * ScaleFactor;
                float uz = -det.robot_xy_mm[1] * ScaleFactor;

                Vector3 robotOrigin = RobotBase != null
                    ? RobotBase.position
                    : Vector3.zero;

                _targetPos[label] = new Vector3(
                    robotOrigin.x + ux,
                    TableY > 0 ? TableY : robotOrigin.y,
                    robotOrigin.z + uz
                );
                obj.SetActive(true);
            }
            else
            {
                obj.SetActive(false);
            }
        }
    }

    // ── Car primitive ─────────────────────────────────────────────────────
    void BuildCar(GameObject root)
    {
        AddPart(root, PrimitiveType.Sphere,
            pos: Vector3.zero,
            scale: new Vector3(0.05f, 0.05f, 0.05f),
            color: Color.red);
    }

    // ── Phone primitive ───────────────────────────────────────────────────

    void BuildPhone(GameObject root)
    {
        // Body
        AddPart(root, PrimitiveType.Cube,
            pos: new Vector3(0f, 0.04f, 0f),
            scale: new Vector3(0.035f, 0.004f, 0.070f),
            color: new Color(0.10f, 0.10f, 0.10f));
        // Screen
        AddPart(root, PrimitiveType.Cube,
            pos: new Vector3(0f, 0.062f, 0f),
            scale: new Vector3(0.028f, 0.0002f, 0.058f),
            color: new Color(0.30f, 0.60f, 1.00f));
        // Home button
        AddPart(root, PrimitiveType.Cylinder,
            pos: new Vector3(0f, 0.062f, -0.28f),
            scale: new Vector3(0.004f, 0.0001f, 0.004f),
            color: new Color(0.40f, 0.40f, 0.40f));
    }

    // ── Part helper ───────────────────────────────────────────────────────

    void AddPart(GameObject root, PrimitiveType type,
                 Vector3 pos, Vector3 scale, Color color,
                 bool rotateZ90 = false)
    {
        var go = GameObject.CreatePrimitive(type);
        go.transform.SetParent(root.transform);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;

        if (rotateZ90)
            go.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);

        var rend = go.GetComponent<Renderer>();
        if (rend != null)
        {
            var mat = new Material(Shader.Find("Standard"));
            mat.color = color;
            rend.material = mat;
        }

        var col = go.GetComponent<Collider>();
        if (col != null) Destroy(col);
    }

    // ── Floating label ────────────────────────────────────────────────────

    void AddFloatingLabel(GameObject root, string label)
    {
        var lo = new GameObject("FloatingLabel");
        lo.transform.SetParent(root.transform);
        lo.transform.localPosition = new Vector3(0f, 0.7f, 0f);
        lo.transform.localScale = Vector3.one * 0.05f;

        var tm = lo.AddComponent<TextMesh>();
        tm.text = label;
        tm.fontSize = 24;
        tm.color = Color.white;
        tm.anchor = TextAnchor.MiddleCenter;
        tm.alignment = TextAlignment.Center;

        lo.AddComponent<FaceCamera>();
    }

    // ═════════════════════════════════════════════════════════════════════
    // Helpers
    // ═════════════════════════════════════════════════════════════════════

    void SetStatus(string text)
    {
        if (StatusText != null) StatusText.text = text;
    }

    IEnumerator FlashStatus(string text)
    {
        SetStatus(text);
        yield return new WaitForSeconds(2.0f);
        int active = 0;
        foreach (var label in TARGET_LABELS)
            if (_buttonMap.ContainsKey(label) &&
                _buttonMap[label].GetComponent<Button>()?.interactable == true)
                active++;
        SetStatus(active > 0 ? $"{active} object(s) detected" : "No target objects detected");
    }

    Color GetColorForLabel(string label)
    {
        switch (label)
        {
            case "car": return new Color(0.53f, 0.81f, 0.98f, 1f);
            case "cell phone": return new Color(0.60f, 0.98f, 0.60f, 1f);
            default:
                int hash = label.GetHashCode();
                float r = ((hash & 0xFF0000) >> 16) / 255f;
                float g = ((hash & 0x00FF00) >> 8) / 255f;
                float b = (hash & 0x0000FF) / 255f;
                return new Color(Mathf.Lerp(r, 1f, .5f), Mathf.Lerp(g, 1f, .5f), Mathf.Lerp(b, 1f, .5f), 1f);
        }
    }
}

/// <summary>Makes a GameObject always face the main camera.</summary>
public class FaceCamera : MonoBehaviour
{
    void LateUpdate()
    {
        if (Camera.main == null) return;
        transform.LookAt(Camera.main.transform);
        transform.Rotate(0f, 180f, 0f);
    }
}