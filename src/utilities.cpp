#include <vector>
#include <string>
#include <keypoint_3d_matching_msgs/Keypoint3d.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>


const std::map<int, std::string> POSE_BODY_65_BODY_PARTS
{
    {0,  "Nose"},
    {1,  "Neck"},
    {2,  "RShoulder"},
    {3,  "RElbow"},
    {4,  "RWrist"},
    {5,  "LShoulder"},
    {6,  "LElbow"},
    {7,  "LWrist"},
    {8,  "MidHip"},
    {9,  "RHip"},
    {10, "RKnee"},
    {11, "RAnkle"},
    {12, "LHip"},
    {13, "LKnee"},
    {14, "LAnkle"},
    {15, "REye"},
    {16, "LEye"},
    {17, "REar"},
    {18, "LEar"},
    {19, "LBigToe"},
    {20, "LSmallToe"},
    {21, "LHeel"},
    {22, "RBigToe"},
    {23, "RSmallToe"},
    {24, "RHeel"},
    {25, "Background"},
    {26, "LThumb1CMC"},
    {27, "LThumb2Knuckles"},
    {28, "LThumb3IP"},
    {29, "LThumb4FingerTip"},
    {30, "LIndex1Knuckles"},
    {31, "LIndex2PIP"},
    {32, "LIndex3DIP"},
    {33, "LIndex4FingerTip"},
    {34, "LMiddle1Knuckles"},
    {35, "LMiddle2PIP"},
    {36, "LMiddle3DIP"},
    {37, "LMiddle4FingerTip"},
    {38, "LRing1Knuckles"},
    {39, "LRing2PIP"},
    {40, "LRing3DIP"},
    {41, "LRing4FingerTip"},
    {42, "LPinky1Knuckles"},
    {43, "LPinky2PIP"},
    {44, "LPinky3DIP"},
    {45, "LPinky4FingerTip"},
    {46, "RThumb1CMC"},
    {47, "RThumb2Knuckles"},
    {48, "RThumb3IP"},
    {49, "RThumb4FingerTip"},
    {50, "RIndex1Knuckles"},
    {51, "RIndex2PIP"},
    {52, "RIndex3DIP"},
    {53, "RIndex4FingerTip"},
    {54, "RMiddle1Knuckles"},
    {55, "RMiddle2PIP"},
    {56, "RMiddle3DIP"},
    {57, "RMiddle4FingerTip"},
    {58, "RRing1Knuckles"},
    {59, "RRing2PIP"},
    {60, "RRing3DIP"},
    {61, "RRing4FingerTip"},
    {62, "RPinky1Knuckles"},
    {63, "RPinky2PIP"},
    {64, "RPinky3DIP"},
    {65, "RPinky4FingerTip"}
};


std::string pointsName(int idx){
    return POSE_BODY_65_BODY_PARTS.find(idx)->second;
}


keypoint_3d_matching_msgs::Keypoint3d_list keypointsStructure(keypoint_3d_matching_msgs::Keypoint3d_list input_msg){
    keypoint_3d_matching_msgs::Keypoint3d point;
    keypoint_3d_matching_msgs::Keypoint3d_list points_v;

    for (int i = 0; i< input_msg.keypoints.size(); i++){
        point.name = input_msg.keypoints[i].name;
        points_v.keypoints.push_back(point);
    }

    return points_v;
}