using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Subscribes to /detections (JSON String) from Layer 3.
/// Parses detections and exposes them for DetectionUI.
/// Also publishes to /robot_command (String) when a button is clicked.
/// </summary>
public class DetectionSubscriber : MonoBehaviour
{
    // 式式 ROS topics 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
    private const string DETECTIONS_TOPIC = "/detections";
    private const string COMMAND_TOPIC = "/robot_command";

    // 式式 Parsed detection data 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
    [Serializable]
    public class Detection
    {
        public string label;
        public float confidence;
        public float[] camera_xyz;    // [x, y, z] in meters (camera frame)
        public float[] robot_xy_mm;   // [x, y] in mm (robot base frame)
        public float robot_z_mm;    // hover Z in mm
    }

    // 式式 Public state (read by DetectionUI) 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
    public List<Detection> CurrentDetections { get; private set; } = new();
    public bool DetectionsUpdated { get; set; } = false;
    public string LastCommand { get; private set; } = "";

    // 式式 ROS connection 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
    private ROSConnection _ros;
    private readonly object _lock = new();

    // 式式 Unity lifecycle 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to detections
        _ros.Subscribe<StringMsg>(DETECTIONS_TOPIC, OnDetections);

        // Register publisher for robot commands
        _ros.RegisterPublisher<StringMsg>(COMMAND_TOPIC);

        Debug.Log($"[DetectionSubscriber] Subscribed to {DETECTIONS_TOPIC}");
        Debug.Log($"[DetectionSubscriber] Publishing to {COMMAND_TOPIC}");
    }

    // 式式 Detections callback 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式

    void OnDetections(StringMsg msg)
    {
        if (string.IsNullOrEmpty(msg.data))
            return;

        try
        {
            // Unity's JsonUtility doesn't support top-level arrays
            // Wrap in an object for parsing
            string wrapped = $"{{\"items\":{msg.data}}}";
            var wrapper = JsonUtility.FromJson<DetectionListWrapper>(wrapped);

            lock (_lock)
            {
                CurrentDetections = wrapper?.items ?? new List<Detection>();
                DetectionsUpdated = true;
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[DetectionSubscriber] JSON parse error: {e.Message}");
        }
    }

    // 式式 Send robot command 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式

    public void SendCommand(string objectLabel)
    {
        if (string.IsNullOrEmpty(objectLabel))
            return;

        var msg = new StringMsg(objectLabel);
        _ros.Publish(COMMAND_TOPIC, msg);

        LastCommand = objectLabel;
        Debug.Log($"[DetectionSubscriber] Published command: '{objectLabel}'");
    }

    // 式式 Get detections (thread-safe) 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式

    public List<Detection> GetDetections()
    {
        lock (_lock)
        {
            return new List<Detection>(CurrentDetections);
        }
    }

    // 式式 JSON wrapper helper 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式

    [Serializable]
    private class DetectionListWrapper
    {
        public List<Detection> items;
    }
}