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
//#include <image_transport/image_transport.h>
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

  ros::Publisher marker_pub_left = nh.advertise<visualization_msgs::Marker>("visualization_marker_left_eq", 10);
  ros::Publisher marker_pub_right = nh.advertise<visualization_msgs::Marker>("visualization_marker_right_eq", 10);

  // 과거 코드로 부터 붙여넣기를 하는 부분
  /*
  float * left_lane_point_x = new float[100];
  float * left_lane_point_y = new float[100];
  float * left_lane_point_z = new float[100]; 

  float * right_lane_point_x = new float[100];
  float * right_lane_point_y = new float[100];
  float * right_lane_point_z = new float[100];
  */
  /*
float left_lane_equat_point_x[] = {0 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 ,39 ,40 ,41 ,42 ,43 ,44 ,45 ,46 ,47 ,48 };
float left_lane_equat_point_y[] = {1.5042456262073003 ,1.4944940898674601 ,1.4857232661162356 ,1.4779331549536268 ,1.471123756379634 ,1.4652950703942567 ,1.4604470969974954 ,1.4565798361893496 ,1.45369328796982 ,1.4517874523389056 ,1.4508623292966074 ,1.4509179188429249 ,1.451954220977858 ,1.453971235701407 ,1.4569689630135718 ,1.4609474029143523 ,1.4659065554037485 ,1.4718464204817605 ,1.4787669981483884 ,1.486668288403632 ,1.4955502912474914 ,1.5054130066799665 ,1.5162564347010574 ,1.528080575310764 ,1.5408854285090865 ,1.5546709942960246 ,1.5694372726715786 ,1.5851842636357483 ,1.6019119671885338 ,1.6196203833299352 ,1.6383095120599522 ,1.657979353378585 ,1.6786299072858335 ,1.7002611737816977 ,1.722873152866178 ,1.746465844539274 ,1.7710392488009856 ,1.796593365651313 ,1.8231281950902563 ,1.850643737117815 ,1.87913999173399 ,1.9086169589387803 ,1.9390746387321864 ,1.9705130311142085 ,2.0029321360848464 ,2.0363319536441002 ,2.0707124837919695 ,2.1060737265284546 ,2.1424156818535556 };
float right_lane_equat_point_x[] = {0 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 ,39 ,40 ,41 ,42 ,43 ,44 ,45 ,46 ,47 ,48 };
float right_lane_equat_point_y[] = {-2.1144090795931727 ,-2.077728823542629 ,-2.0417160658553533 ,-2.0063708065313444 ,-1.9716930455706034 ,-1.9376827829731296 ,-1.9043400187389234 ,-1.8716647528679846 ,-1.8396569853603133 ,-1.8083167162159093 ,-1.777643945434773 ,-1.747638673016904 ,-1.7183008989623025 ,-1.6896306232709684 ,-1.6616278459429017 ,-1.6342925669781025 ,-1.6076247863765707 ,-1.5816245041383064 ,-1.5562917202633095 ,-1.5316264347515802 ,-1.5076286476031182 ,-1.4842983588179237 ,-1.4616355683959967 ,-1.439640276337337 ,-1.4183124826419449 ,-1.3976521873098202 ,-1.377659390340963 ,-1.3583340917353732 ,-1.339676291493051 ,-1.321685989613996 ,-1.3043631860982086 ,-1.2877078809456886 ,-1.2717200741564358 ,-1.2563997657304509 ,-1.2417469556677332 ,-1.227761643968283 ,-1.2144438306321002 ,-1.201793515659185 ,-1.189810699049537 ,-1.1784953808031566 ,-1.1678475609200438 ,-1.1578672394001983 ,-1.1485544162436203 ,-1.1399090914503098 ,-1.1319312650202664 ,-1.124620936953491 ,-1.1179781072499826 ,-1.1120027759097417 ,-1.1066949429327684 };
  */
float left_lane_equat_point_x[] = {0 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 };
float left_lane_equat_point_y[] = {2.3410404240722693 ,2.3360578757218886 ,2.331534346016322 ,2.3274698349555694 ,2.3238643425396304 ,2.320717868768505 ,2.3180304136421936 ,2.3158019771606964 ,2.3140325593240125 ,2.312722160132143 ,2.311870779585087 ,2.3114784176828453 ,2.311545074425417 ,2.312070749812803 ,2.3130554438450024 ,2.314499156522016 ,2.3164018878438433 ,2.318763637810484 ,2.3215844064219393 ,2.3248641936782084 ,2.328602999579291 ,2.3328008241251874 ,2.3374576673158978 ,2.3425735291514225 ,2.3481484096317606 ,2.3541823087569123 ,2.3606752265268782 ,2.367627162941658 ,2.375038118001252 ,2.382908091705659 ,2.3912370840548807 ,2.4000250950489157 ,2.4092721246877646 ,2.4189781729714275 ,2.429143239899904 ,2.439767325473195 ,2.4508504296912994 ,2.462392552554218 ,2.47439369406195 };
float right_lane_equat_point_x[] = {0 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 };
float right_lane_equat_point_y[] = {-2.2894339394472394 ,-2.270126919815131 ,-2.25155973745761 ,-2.2337323923746757 ,-2.2166448845663287 ,-2.200297214032569 ,-2.184689380773396 ,-2.16982138478881 ,-2.1556932260788115 ,-2.1423049046433995 ,-2.129656420482575 ,-2.1177477735963377 ,-2.1065789639846875 ,-2.096149991647624 ,-2.086460856585148 ,-2.0775115587972586 ,-2.0693020982839565 ,-2.0618324750452417 ,-2.0551026890811137 ,-2.049112740391573 ,-2.043862628976619 ,-2.0393523548362524 ,-2.0355819179704726 ,-2.0325513183792805 ,-2.0302605560626747 ,-2.0287096310206567 ,-2.0278985432532255 ,-2.027827292760381 ,-2.028495879542124 ,-2.029904303598454 ,-2.032052564929371 ,-2.0349406635348752 ,-2.0385685994149663 ,-2.0429363725696446 ,-2.04804398299891 ,-2.0538914307027625 ,-2.060478715681202 ,-2.067805837934229 ,-2.0758727974618427 };


  float height = -1.50115;
  int len_lane_pixel = sizeof(left_lane_equat_point_x)/ sizeof(left_lane_equat_point_x[0]);
  cout << len_lane_pixel << endl;
  ros::Rate loop_rate(4);
  
  while (nh.ok())
  {
    visualization_msgs::Marker left_points, left_line_strip, left_line_list;
    left_points.header.frame_id = left_line_strip.header.frame_id = left_line_list.header.frame_id = "livox_frame";
    left_points.header.stamp = left_line_strip.header.stamp = left_line_list.header.stamp = ros::Time::now();
    left_points.ns = left_line_strip.ns = left_line_list.ns = "points_and_lines_eq";
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
      p_l.z = height;                            // z 값은 없으므로 1 로 한다.
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
    right_points.ns = right_line_strip.ns = right_line_list.ns = "points_and_lines_eq";
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
      p_r.z = height;
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
