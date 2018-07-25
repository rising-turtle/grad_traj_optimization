/*
    July 24 2018, He Zhang, hzhang8@vcu.edu 
    
    read PCD file, and then generate a path by clicking 

*/

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <stdlib.h>
#include <string>
#include "display.h"
#include "grad_traj_optimizer.h"

using namespace std;

ros::Subscriber waypoint_sub;
void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

bool waypoint_enough = false;
vector<Eigen::Vector3d> way_points;

void pcd_click(string pcd_file); 
void usingGivenWaypoints(); 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pcd_click"); 
    ros::NodeHandle node; 
    
    if(argc <= 1) 
    {
	ROS_INFO("usage: pcd_click *.pcd");
	return 0;
    }

    pcd_click(string(argv[1])); 

    return 1; 

}

void pcd_click(string pcd_file)
{
    // 1. read pcd 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::io::loadPCDFile(pcd_file, *pc); 

    // 2. set visualize msg 
    ros::NodeHandle node; 
    setpoint_pub = node.advertise<visualization_msgs::Marker>("trajopt/setpoint", 10);
    traj_point_pub = node.advertise<visualization_msgs::Marker>("trajopt/traj_point", 10);
    traj_pub = node.advertise<nav_msgs::Path>("trajopt/init_traj", 5);
    ros::Publisher visualization_pub =
	node.advertise<visualization_msgs::Marker>("sdf_tools_tutorial_visualization", 1, true);
    
    waypoint_sub = node.subscribe("/move_base_simple/goal", 5, waypointCallback);  // 2D Nav Goal
    // waypoint_sub= node.subscribe("/goal", 5, waypointCallback);                   // 3D Nav Goal

    srand(ros::Time::now().toSec());
    ros::Duration(0.5).sleep();
    point_num = 7; 
    ros::param::get("/traj_opti_node1/point_num", point_num);
    
    // 3. sdf collision map parameter
    const double resolution = 0.05; // 0.1;
    const double x_size = 20.; // 20.0;
    const double z_size = 5.0; // 5.0;
    double y_size = 20.; // 20.0;
    Eigen::Translation3d origin_translation(-10., -10., 0.); //(-10.0, -10.0, 0.0);
    Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
    const Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
    const std ::string frame = "world";

    // 4. create a collision map using pcd 
    sdf_tools ::COLLISION_CELL cell;
    cell.occupancy = 0.0;
    cell.component = 0;
    const sdf_tools ::COLLISION_CELL oob_cell = cell;
    sdf_tools ::CollisionMapGrid collision_map(origin_transform, frame, resolution, x_size, y_size,
	    z_size, oob_cell);

    // add some obstacle randomly
    sdf_tools::COLLISION_CELL obstacle_cell(1.0);

    // add obstacle point 
    for(int i=0; i<pc->points.size(); i++)
    {
	pcl::PointXYZRGB & pt = pc->points[i]; 
	collision_map.Set(pt.x, pt.y, pt.z, obstacle_cell); 
    }
    
    // visualize the collision map 
    std_msgs::ColorRGBA collision_color;
    collision_color.r = 0.0;
    collision_color.g = 0.0;
    collision_color.b = 1.0;
    collision_color.a = 0.8;

    std_msgs::ColorRGBA free_color, unknown_color;
    unknown_color.a = free_color.a = 0.0;

    visualization_msgs::Marker collision_map_marker =
	collision_map.ExportForDisplay(collision_color, free_color, unknown_color);
    collision_map_marker.ns = "collision_map";
    collision_map_marker.id = 1;

    visualization_pub.publish(collision_map_marker);
    
    // Build the signed distance field
    float oob_value = INFINITY;
    std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf_with_extrema =
	collision_map.ExtractSignedDistanceField(oob_value);

    sdf_tools::SignedDistanceField sdf = sdf_with_extrema.first;
    cout << "----------------------Signed distance field build!----------------------" << endl;

    //-----------------------------Wait for user to click waypoint--------------------
    cout << "----------------------Please click some way_points----------------------- " << endl;

    // test with given points
    // usingGivenWaypoints(); 
    // ros::spinOnce(); 

    while(ros::ok() && !waypoint_enough)
    {
	ros::spinOnce();
	ros::Duration(0.5).sleep();
    }

    // ----------------------------main optimization procedure--------------------------
    GradTrajOptimizer grad_traj_opt(node, way_points);
    grad_traj_opt.setSignedDistanceField(&sdf, resolution);

    Eigen::MatrixXd coeff;
    grad_traj_opt.getCoefficient(coeff);
    grad_traj_opt.getSegmentTime(my_time);
    displayTrajectory(coeff, false);

    // first step optimization
    grad_traj_opt.optimizeTrajectory(OPT_FIRST_STEP);
    grad_traj_opt.getCoefficient(coeff);
    displayTrajectory(coeff, false);

    //  second step optimization
    grad_traj_opt.optimizeTrajectory(OPT_SECOND_STEP);
    grad_traj_opt.getCoefficient(coeff);
    displayTrajectory(coeff, true);

    return ;
}

void usingGivenWaypoints()
{
    Eigen::Vector3d pt; 
    pt(2) = 0.3; // 2

    pt(0) = 0.880678; 
    pt(1) = 1.03086; 
    way_points.push_back(pt); 

    pt(0) = 0.624083; // 0.498248;      
    pt(1) = 1.71424;  // 0.326108;
    way_points.push_back(pt); 

    pt(0) = 0.194154; // -0.520836; 
    pt(1) = 2.08111;  // 0.301035;
    way_points.push_back(pt); 

    pt(0) = -0.401702; // -0.987531;  
    pt(1) = 1.89875; // 1.12145; 
    way_points.push_back(pt); 

    pt(0) = -0.757012; //-1.58797;  
    pt(1) = 2.64474; // 1.76864; 
    way_points.push_back(pt); 

    pt(0) =  -0.983396; // -1.66106;  
    pt(1) = 3.15419; // 2.63501; 
    way_points.push_back(pt); 

    pt(0) = -1.02855;
    pt(1) = 3.5216; 
    way_points.push_back(pt); 
 
    visualizeSetPoints(way_points);
    waypoint_enough = true; 
}


void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(msg->pose.position.z < -0.01 || msg->pose.position.z > 4.0)
  {
    ROS_WARN("z should be between 0.0 and 4.0!!");
    return;
  }

  Eigen::Vector3d pt;
  pt(0) = msg->pose.position.x;
  pt(1) = msg->pose.position.y;
  // pt(2)= msg->pose.position.z;
  pt(2) = 0.2; // 2

  way_points.push_back(pt);

  visualizeSetPoints(way_points);
  cout << "------------------New waypoint added!------------------" << endl; 
  cout << " pt: "<<pt(0)<<" "<<pt(1)<<endl; 

  if(way_points.size() == point_num)
  {
    cout << "------------------Waypoint enough!------------------" << endl;
    waypoint_enough = true;
  }
  else
  {
    cout << "------------------" << (point_num - way_points.size()) << " more way_points required"
         << endl;
  }
}
