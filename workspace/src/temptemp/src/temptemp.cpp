#include<ros/ros.h>
#include <vector>

#include<std_msgs/Float32.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

nav_msgs::Path sort_path;
geometry_msgs::PoseStamped sort_pose;

ros::Publisher sort_yolo_3d_coords_pub;

void myCallback(std_msgs::Float32 msg){
    //cout<<" data ="<<msg.data<<endl; 
}
void myCallback2(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    int detection_count = 0;
    int center_x, center_y;

    //cout<<" bboxes_data = " << msg->bounding_boxes[detection_count] <<endl; 
    //cout << "bboxes_Count = " << msg->bounding_boxes.size() << endl;

    for(int i = 0;i< msg->bounding_boxes.size();i++)
    {
        center_x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax)/2;
        center_y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax)/2;
        //cout << "Center Point = (" << center_x << " , " << center_y << ")" << endl;
    }

}
void myCallback3(const darknet_ros_msgs::ObjectCount::ConstPtr& msg){
    //cout<<" bbox_Count = " << msg->count <<endl; 
    //cout << "---------------------------" << endl;
}

void Sort_3d_Coord(const nav_msgs::Path::ConstPtr& msg){

    vector<float> z_list;

    //cout << "---------------------------" <<endl;
    //cout << "3D Coord = " << msg->poses[0].pose.position.x <<endl;
    cout << "---------------------------" <<endl;


    //Indexing 작업 
    cout << msg->poses.size() <<endl;

    // Check2 -> Z Value list에 저장 
    for(int i =0;i<msg->poses.size();i++)
    {
        z_list.push_back(msg->poses[i].pose.position.x);
    }
    // Z value 정렬
    sort(z_list.begin(),z_list.end());

    // Z 정렬된 것으로 nav_msgs/Path 에 저장
    for (int k =0; k < z_list.size(); k++)
    {
        cout << z_list[k] << endl;
        for (int j =0; j < z_list.size(); j++)
        {
            if(z_list[k] == msg->poses[j].pose.position.x && msg->poses[j].pose.position.y >= -0.15 && msg->poses[j].pose.position.y <= 0.15)
            {
                sort_path.poses.push_back(msg->poses[j]);
                cout << "Find Same & Detect ROI position" << endl;
            }

        }
    }

    sort_yolo_3d_coords_pub.publish(sort_path);
    sort_path.poses.clear();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "temptemp");
    ros::NodeHandle nh;

    ros::Rate l(20);
    ros::Subscriber abc = nh.subscribe("/my_msg",10 ,myCallback);
    ros::Subscriber yolo_bbox_boxes_sub = nh.subscribe("/darknet_ros/bounding_boxes",10 ,myCallback2);
    ros::Subscriber yolo_bbox_box_sub = nh.subscribe("/darknet_ros/found_object",10 ,myCallback3);
    ros::Subscriber yolo_3d_coords_sub = nh.subscribe("/darknet_3d/check2",10,Sort_3d_Coord);
    //ros::Subscriber check_recieve_3d_sub = nh.subscribe("/dar")

    sort_yolo_3d_coords_pub = nh.advertise<nav_msgs::Path>("/darknet_3d/sorted_poses",100);



    while(ros::ok()){
        l.sleep();
        ros::spinOnce();
        //cout<<"In LOOP"<<endl;
    }
    return 0;
}
