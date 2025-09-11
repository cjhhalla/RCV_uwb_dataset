#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <cmath>

class DronePoseVisualizer {
public:
    DronePoseVisualizer() : line_length(0.0) {
        ros::NodeHandle nh;

        pose_sub = nh.subscribe("mavros/vision_pose/pose", 10, &DronePoseVisualizer::poseCallback, this);
        drone_sub = nh.subscribe("mavros/local_position/pose", 10, &DronePoseVisualizer::mavrosCallback, this);
        uwb_sub = nh.subscribe("UWB_range",10,&DronePoseVisualizer::rangeCallback,this);
        // anchor1_pose_sub = nh.subscribe("/qualisys/anchor1/pose",10,&DronePoseVisualizer::anchorPoseCallback,this);
        // anchor2_pose_sub = nh.subscribe("/qualisys/anchor2/pose",10,&DronePoseVisualizer::anchorPoseCallback,this);
        // anchor3_pose_sub = nh.subscribe("/qualisys/anchor3/pose",10,&DronePoseVisualizer::anchorPoseCallback,this);
        // anchor_pose_pub = nh.advertise<visualization_msgs::Marker>("anchor_marker",10,true);

        path_pub = nh.advertise<nav_msgs::Path>("drone_path", 10, true);
        path_local_pub = nh.advertise<nav_msgs::Path>("drone_path_local", 10, true);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("orientation_markers", 10, true);


        path.header.frame_id = "world";  
        path_drone.header.frame_id = "world";
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!std::isnan(msg->pose.position.x) && !std::isnan(msg->pose.position.y) && !std::isnan(msg->pose.position.z) &&
        !std::isnan(msg->pose.orientation.x) && !std::isnan(msg->pose.orientation.y) &&
        !std::isnan(msg->pose.orientation.z) && !std::isnan(msg->pose.orientation.w)) {
        path.poses.push_back(*msg);
        path_pub.publish(path);
        updateOrientationMarkers(*msg);
        }
        else 
        {
        ROS_WARN("Mocap_pos is NAN.");
        }
    }
    void mavrosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!std::isnan(msg->pose.position.x) && !std::isnan(msg->pose.position.y) && !std::isnan(msg->pose.position.z) &&
        !std::isnan(msg->pose.orientation.x) && !std::isnan(msg->pose.orientation.y) &&
        !std::isnan(msg->pose.orientation.z) && !std::isnan(msg->pose.orientation.w)) 
        {
        path_drone.poses.push_back(*msg);
        path_local_pub.publish(path_drone);
        }
    }

    void anchorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!std::isnan(msg->pose.position.x) && !std::isnan(msg->pose.position.y) && !std::isnan(msg->pose.position.z) &&
        !std::isnan(msg->pose.orientation.x) && !std::isnan(msg->pose.orientation.y) &&
        !std::isnan(msg->pose.orientation.z) && !std::isnan(msg->pose.orientation.w)) 
        {
            genMarker(*msg);
        }
    }

    void rangeCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        line_length = msg->data;
    }


private:
    ros::Subscriber uwb_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber drone_sub;
    ros::Subscriber anchor1_pose_sub;
    ros::Subscriber anchor2_pose_sub;
    ros::Subscriber anchor3_pose_sub;
    ros::Publisher anchor_pose_pub;
    
    ros::Publisher path_pub;
    ros::Publisher path_local_pub;
    ros::Publisher marker_pub;
    ros::Publisher line_pub;
    nav_msgs::Path path;
    nav_msgs::Path path_drone;

    double line_length;

    void genMarker(const geometry_msgs::PoseStamped& pose)
    {
        visualization_msgs::Marker anchor_m;
        anchor_m.header.frame_id = "world";
        anchor_m.header.stamp = ros::Time::now();
        anchor_m.ns = "anchor marker";
        anchor_m.id = 0;
        anchor_m.type = visualization_msgs::Marker::CUBE;
        anchor_m.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point marker;
        marker.x = pose.pose.position.x;
        marker.y = pose.pose.position.y;
        marker.z = pose.pose.position.z;
        
        anchor_m.points.push_back(marker);
        anchor_pose_pub.publish(marker);
    }

    void updateRange(const geometry_msgs::PoseStamped& pose)
    {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "world";  // 적절한 frame_id 설정
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "distance_line";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        double normal_vec = sqrt(pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2) + pow(pose.pose.position.z, 2));

        double dir_x = pose.pose.position.x / normal_vec;
        double dir_y = pose.pose.position.y / normal_vec;
        double dir_z = pose.pose.position.z / normal_vec;
        // 선의 시작점(원점)과 끝점(드론의 현재 위치) 설정
        geometry_msgs::Point start, end;
        start.x = 0; start.y = 0; start.z = 0; // 원점
        end.x = dir_x * line_length;
        end.y = dir_y * line_length;
        end.z = dir_z * line_length;
    
        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        // 선의 속성 설정
        line_marker.scale.x = 0.01; // 선의 굵기
        line_marker.color.r = 1.0; line_marker.color.g = 0.0;
        line_marker.color.b = 0.0; line_marker.color.a = 1.0; // 선의 색상(빨간색)

        line_pub.publish(line_marker);
    }

    void updateOrientationMarkers(const geometry_msgs::PoseStamped& pose) {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers = createOrientationMarkers(pose.pose, pose.header.frame_id, pose.header.stamp);

        marker_pub.publish(marker_array);
    }

    std::vector<visualization_msgs::Marker> createOrientationMarkers(const geometry_msgs::Pose& pose, const std::string& frame_id, const ros::Time& time) {
        std::vector<visualization_msgs::Marker> markers;
        std::string axes = "xyz";
        float colors[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // RGB

        tf::Quaternion drone_orientation;
        tf::quaternionMsgToTF(pose.orientation, drone_orientation);

        for (int i = 0; i < 3; ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = time;
            marker.ns = "orientation_axis";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = 0.5; // 화살표 길이
            marker.scale.y = 0.1; // 화살표 너비
            marker.scale.z = 0.1; // 화살표 높이
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.color.a = 1.0;

            // 각 축 방향으로 화살표 조정
            tf::Quaternion axis_rotation;
            switch (i) {
                case 0: axis_rotation = tf::createQuaternionFromRPY(0, 0, 0); break; // x축
                case 1: axis_rotation = tf::createQuaternionFromRPY(0, 0, M_PI / 2); break; // y축
                case 2: axis_rotation = tf::createQuaternionFromRPY(0, -M_PI / 2, 0); break; // z축
            }
            tf::Quaternion combined_orientation = drone_orientation * axis_rotation;
            combined_orientation.normalize();

            // 결합된 오리엔테이션을 geometry_msgs::Quaternion으로 변환
            geometry_msgs::Quaternion orientation;
            tf::quaternionTFToMsg(combined_orientation, orientation);

            // 마커의 오리엔테이션 설정
            marker.pose.orientation = orientation;

            markers.push_back(marker);
        }

        return markers;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_pose_visualizer");
    DronePoseVisualizer visualizer;
    ros::spin();
    return 0;
}
