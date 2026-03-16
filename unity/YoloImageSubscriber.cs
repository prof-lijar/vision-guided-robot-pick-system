using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class YoloImageSubscriber : MonoBehaviour
{
    public string imageTopic = "/yolo/detected_image";
    public RawImage displayImage; // drag your RawImage here

    private Texture2D texture;

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<ImageMsg>(imageTopic, OnImageReceived);
    }

    void OnImageReceived(ImageMsg msg)
    {
        int w = (int)msg.width;
        int h = (int)msg.height;

        if (texture == null || texture.width != w || texture.height != h)
            texture = new Texture2D(w, h, TextureFormat.RGB24, false);

        // Swap BGR ¡æ RGB
        byte[] src = msg.data;
        byte[] rgb = new byte[src.Length];
        for (int i = 0; i < src.Length; i += 3)
        {
            rgb[i] = src[i + 2];
            rgb[i + 1] = src[i + 1];
            rgb[i + 2] = src[i];
        }

        texture.LoadRawTextureData(rgb);
        texture.Apply();

        displayImage.texture = texture;
        displayImage.uvRect = new Rect(0, 1, 1, -1); // flip Y (ROS convention)
    }
}