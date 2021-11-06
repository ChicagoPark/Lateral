#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
//#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
using namespace std;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "Lane_Marker");
  ros::NodeHandle nh;
  double x_cloud; double y_cloud; double z_cloud;

  ros::Publisher marker_pub_left = nh.advertise<visualization_msgs::Marker>("visualization_marker_left", 10);
  ros::Publisher marker_pub_right = nh.advertise<visualization_msgs::Marker>("visualization_marker_right", 10);

  // 과거 코드로 부터 붙여넣기를 하는 부분
  /*
  float * left_lane_point_x = new float[100];
  float * left_lane_point_y = new float[100];
  float * left_lane_point_z = new float[100]; 

  float * right_lane_point_x = new float[100];
  float * right_lane_point_y = new float[100];
  float * right_lane_point_z = new float[100];
  */
  float left_lane_equat_point_x[] = {0 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 ,39 ,40 ,41 ,42 ,43 ,44 ,45 ,46 ,47 ,48 ,49 ,50 ,51 ,52 ,53 ,54 ,54 };
  float left_lane_equat_point_y[] = {1.2184881212680339 ,1.2234946770642174 ,1.22865057598572 ,1.233955818032542 ,1.239410403204683 ,1.2450143315021431 ,1.2507676029249226 ,1.256670217473021 ,1.2627221751464388 ,1.2689234759451757 ,1.2752741198692317 ,1.2817741069186068 ,1.2884234370933012 ,1.2952221103933148 ,1.3021701268186474 ,1.3092674863692992 ,1.3165141890452703 ,1.3239102348465606 ,1.3314556237731698 ,1.3391503558250983 ,1.346994431002346 ,1.3549878493049128 ,1.363130610732799 ,1.3714227152860041 ,1.3798641629645283 ,1.388454953768372 ,1.3971950876975345 ,1.4060845647520164 ,1.4151233849318174 ,1.4243115482369375 ,1.4336490546673768 ,1.4431359042231353 ,1.452772096904213 ,1.4625576327106098 ,1.4724925116423258 ,1.482576733699361 ,1.4928102988817153 ,1.5031932071893888 ,1.5137254586223814 ,1.5244070531806933 ,1.535237990864324 ,1.5462182716732742 ,1.5573478956075437 ,1.568626862667132 ,1.5800551728520398 ,1.5916328261622665 ,1.6033598225978125 ,1.6152361621586777 ,1.627261844844862 ,1.6394368706563656 ,1.651761239593188 ,1.6642349516553299 ,1.6768580068427907 ,1.689630405155571 ,1.7025521465936704 ,1.7025521465936704 };
  float right_lane_equat_point_x[] = {0 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 ,39 ,40 ,41 ,42 ,43 ,44 ,45 ,46 ,47 ,48 ,49 ,50 ,51 ,52 ,53 ,54 ,54 };
  float right_lane_equat_point_y[] = {-1.800654410784622 ,-1.7858535165122043 ,-1.7711158534469436 ,-1.75644142158884 ,-1.7418302209378935 ,-1.7272822514941042 ,-1.712797513257472 ,-1.698376006227997 ,-1.6840177304056791 ,-1.6697226857905183 ,-1.6554908723825148 ,-1.6413222901816682 ,-1.627216939187979 ,-1.6131748194014468 ,-1.5991959308220716 ,-1.5852802734498537 ,-1.571427847284793 ,-1.5576386523268893 ,-1.5439126885761427 ,-1.5302499560325533 ,-1.516650454696121 ,-1.5031141845668459 ,-1.489641145644728 ,-1.4762313379297671 ,-1.4628847614219633 ,-1.4496014161213169 ,-1.4363813020278273 ,-1.4232244191414951 ,-1.4101307674623198 ,-1.3971003469903018 ,-1.384133157725441 ,-1.3712291996677373 ,-1.3583884728171904 ,-1.345610977173801 ,-1.3328967127375688 ,-1.3202456795084936 ,-1.3076578774865752 ,-1.2951333066718145 ,-1.2826719670642106 ,-1.270273858663764 ,-1.2579389814704744 ,-1.2456673354843422 ,-1.2334589207053668 ,-1.2213137371335487 ,-1.2092317847688876 ,-1.1972130636113838 ,-1.185257573661037 ,-1.1733653149178473 ,-1.1615362873818151 ,-1.1497704910529398 ,-1.1380679259312214 ,-1.1264285920166606 ,-1.1148524893092566 ,-1.1033396178090098 ,-1.0918899775159203 ,-1.0918899775159203 };
  int len_lane_pixel = sizeof(left_lane_equat_point_x)/ sizeof(left_lane_equat_point_x[0]);
  cout << len_lane_pixel << endl;
  ros::Rate loop_rate(4);
  
  while (nh.ok())
  {
    visualization_msgs::Marker left_points, left_line_strip, left_line_list;
    left_points.header.frame_id = left_line_strip.header.frame_id = left_line_list.header.frame_id = "livox_frame";
    left_points.header.stamp = left_line_strip.header.stamp = left_line_list.header.stamp = ros::Time::now();
    left_points.ns = left_line_strip.ns = left_line_list.ns = "points_and_lines";
    left_points.action = left_line_strip.action = left_line_list.action = visualization_msgs::Marker::ADD;
    left_points.pose.orientation.w = left_line_strip.pose.orientation.w = left_line_list.pose.orientation.w = 1.0;
    left_points.id = 0;
    left_line_strip.id = 1;
    left_line_list.id = 2;
    left_points.type = visualization_msgs::Marker::POINTS;  // POINTS 를 SPHERE 로 바꿔보자 나중에
    left_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    left_line_list.type = visualization_msgs::Marker::LINE_LIST;

    left_points.scale.x = 0.2;
    left_points.scale.y = 0.2;
    left_line_strip.scale.x = 0.1;
    left_line_list.scale.x = 0.1;

    // Points are green
    left_points.color.g = 1.0f;
    left_points.color.a = 1.0;

    // Line strip is blue
    left_line_strip.color.b = 1.0;
    left_line_strip.color.a = 1.0;
    
    // Line list is red
    left_line_list.color.r = 1.0;
    left_line_list.color.a = 1.0;
    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < len_lane_pixel; ++i)
    {
      geometry_msgs::Point p_l;
      p_l.x = left_lane_equat_point_x[i];
      p_l.y = left_lane_equat_point_y[i];
      p_l.z = 1;                            // z 값은 없으므로 1 로 한다.
      left_points.points.push_back(p_l);
      left_line_strip.points.push_back(p_l); 
      // The line list needs two points for each line
      left_line_list.points.push_back(p_l);
      p_l.z += 1.0;
      left_line_list.points.push_back(p_l);
  }

    visualization_msgs::Marker right_points, right_line_strip, right_line_list;
    right_points.header.frame_id = right_line_strip.header.frame_id = right_line_list.header.frame_id = "livox_frame";
    right_points.header.stamp = right_line_strip.header.stamp = right_line_list.header.stamp = ros::Time::now();
    right_points.ns = right_line_strip.ns = right_line_list.ns = "points_and_lines";
    right_points.action = right_line_strip.action = right_line_list.action = visualization_msgs::Marker::ADD;
    right_points.pose.orientation.w = right_line_strip.pose.orientation.w = right_line_list.pose.orientation.w = 1.0;
    right_points.id = 0;
    right_line_strip.id = 1;
    right_line_list.id = 2;
    right_points.type = visualization_msgs::Marker::POINTS;  // POINTS 를 SPHERE 로 바꿔보자 나중에
    right_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    right_line_list.type = visualization_msgs::Marker::LINE_LIST;

    right_points.scale.x = 0.2;
    right_points.scale.y = 0.2;
    right_line_strip.scale.x = 0.1;
    right_line_list.scale.x = 0.1;

    // Points are green
    right_points.color.g = 1.0f;
    right_points.color.a = 1.0;

    // Line strip is blue
    right_line_strip.color.b = 1.0;
    right_line_strip.color.a = 1.0;
    
    // Line list is red
    right_line_list.color.r = 1.0;
    right_line_list.color.a = 1.0;
    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < len_lane_pixel; ++i)
    {

      geometry_msgs::Point p_r;
      p_r.x = right_lane_equat_point_x[i];
      p_r.y = right_lane_equat_point_y[i];
      p_r.z = 1;
      right_points.points.push_back(p_r);
      right_line_strip.points.push_back(p_r); 
      // The line list needs two points for each line
      right_line_list.points.push_back(p_r);
      p_r.z += 1.0;
      right_line_list.points.push_back(p_r);
  }


    marker_pub_left.publish(left_points);
    marker_pub_left.publish(left_line_strip);
    marker_pub_left.publish(left_line_list);
    marker_pub_right.publish(right_points);
    marker_pub_right.publish(right_line_strip);
    marker_pub_right.publish(right_line_list);

    ros::spinOnce ();
    loop_rate.sleep ();

    

  }
}
