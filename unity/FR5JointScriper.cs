using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;  // StringMsg

public class FR5JointSubscriber : MonoBehaviour
{
    ROSConnection ros;

    public Transform joint1; // RobotArm_01_02
    public Transform joint2; // RobotArm_01_03
    public Transform joint3; // RobotArm_01_04
    public Transform joint4; // RobotArm_01_06
    public Transform joint5; // RobotArm_01_07
    public Transform joint6; // RobotArm_01_08

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>("/nonrt_state_data", OnStateData);
    }

    void OnStateData(StringMsg msg)
    {
        RobotState state = JsonUtility.FromJson<RobotState>(msg.data);
        if (state == null || state.jt_cur_pos == null || state.jt_cur_pos.Length < 6)
            return;

        // jt_cur_pos is already in degrees — same rotation axes as before
        float j1 = state.jt_cur_pos[0];
        float j2 = state.jt_cur_pos[1];
        float j3 = state.jt_cur_pos[2];
        float j4 = state.jt_cur_pos[3];
        float j5 = state.jt_cur_pos[4];
        float j6 = state.jt_cur_pos[5];

        joint1.localRotation = Quaternion.Euler(0, -j1, 0);
        joint2.localRotation = Quaternion.Euler(0, 0, -j2);
        joint3.localRotation = Quaternion.Euler(0, 0, -j3);
        joint4.localRotation = Quaternion.Euler(0, 0, -j4);
        joint5.localRotation = Quaternion.Euler(0, j5, 0);

        Quaternion restPose = Quaternion.Euler(-16.94f, -90f, -90f);
        Quaternion spin = Quaternion.AngleAxis(-j6, Vector3.up);
        joint6.localRotation = restPose * spin;
    }

    // Matches the JSON keys from state_publisher.py
    [System.Serializable]
    class RobotState
    {
        public int program_state;
        public int robot_state;
        public int robot_mode;
        public float[] jt_cur_pos;
        public float[] tl_cur_pos;
        public bool emergency_stop;
        public bool collision_state;
    }
}