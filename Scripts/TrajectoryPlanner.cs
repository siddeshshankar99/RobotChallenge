using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }

    [SerializeField]
    GameObject m_Target1;
    public GameObject Target1 { get => m_Target1; set => m_Target1 = value; }
    [SerializeField]
    GameObject m_Target2;
    public GameObject Target2 { get => m_Target2; set => m_Target2 = value; }
    [SerializeField]
    GameObject m_Target3; // New target
    public GameObject Target3 { get => m_Target3; set => m_Target3 = value; }
    [SerializeField]
    GameObject m_Target4; // New target
    public GameObject Target4 { get => m_Target4; set => m_Target4 = value; }
    [SerializeField]
    GameObject m_Target5; // New target
    public GameObject Target5 { get => m_Target5; set => m_Target5 = value; }
    [SerializeField]
    GameObject m_Target6; // New target
    public GameObject Target6 { get => m_Target6; set => m_Target6 = value; }
    [SerializeField]
    GameObject m_Target7; // New target
    public GameObject Target7 { get => m_Target7; set => m_Target7 = value; }
    

    [SerializeField]
    GameObject m_TargetPlacement1;
    public GameObject TargetPlacement1 { get => m_TargetPlacement1; set => m_TargetPlacement1 = value; }
    [SerializeField]
    GameObject m_TargetPlacement2;
    public GameObject TargetPlacement2 { get => m_TargetPlacement2; set => m_TargetPlacement2 = value; }
    [SerializeField]
    GameObject m_TargetPlacement3;
    public GameObject TargetPlacement3 { get => m_TargetPlacement3; set => m_TargetPlacement3 = value; }
    [SerializeField]
    GameObject m_TargetPlacement4;
    public GameObject TargetPlacement4 { get => m_TargetPlacement4; set => m_TargetPlacement4 = value; }
    [SerializeField]
    GameObject m_TargetPlacement5;
    public GameObject TargetPlacement5 { get => m_TargetPlacement5; set => m_TargetPlacement5 = value; }
    [SerializeField]
    GameObject m_TargetPlacement6;
    public GameObject TargetPlacement6 { get => m_TargetPlacement6; set => m_TargetPlacement6 = value; }
    [SerializeField]
    GameObject m_TargetPlacement7;
    public GameObject TargetPlacement7 { get => m_TargetPlacement7; set => m_TargetPlacement7 = value; }
    


    // Assures that the gripper is always positioned above the target cubes before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    // Additional variable to keep track of the current target
    private GameObject mCurrentTarget;
    private int mTargetIndex = 0;
    private GameObject[] mTargets;
    private GameObject[] mTargetPlacements;
    // Variable to store the initial joint configuration
    private NiryoMoveitJointsMsg m_InitialJointConfig;


    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        // Initialize targets array
        mTargets = new GameObject[] { m_Target1, m_Target2, m_Target3, m_Target4, m_Target5, m_Target6, m_Target7};
        mTargetPlacements = new GameObject[] { m_TargetPlacement1, m_TargetPlacement2, m_TargetPlacement3, m_TargetPlacement4, m_TargetPlacement5, m_TargetPlacement6, m_TargetPlacement7};
    }

    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    public void PublishJoints()
    {
        StartCoroutine(SequentialPickAndPlace());
    }

    IEnumerator SequentialPickAndPlace()
    {
        for (int i = 0; i < mTargets.Length; i++)
        {
            mCurrentTarget = mTargets[i];
            PublishJointsForTarget(mCurrentTarget, mTargetPlacements[i]);
            // Wait until the current target's task is completed before moving to the next one
            yield return new WaitUntil(() => mTargetIndex != i);
        }
    }

    private void PublishJointsForTarget(GameObject target, GameObject targetPlacement)
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose for the target
        request.pick_pose = new PoseMsg
        {
            position = (target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose for the corresponding target placement
        request.place_pose = new PoseMsg
        {
            position = (targetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
            // Increment the target index to move to the next target
            mTargetIndex++;
        }
    }

    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            OpenGripper();

            yield return new WaitForSeconds(k_PoseAssignmentWait);

            mTargetIndex++;
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}
