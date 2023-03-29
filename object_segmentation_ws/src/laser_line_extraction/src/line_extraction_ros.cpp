#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>
#include <ros/console.h>


namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
  nh_(nh),
  nh_local_(nh_local),
  data_cached_(false),
  tf_cached_(false)
{
  loadParameters();
  line_publisher_ = nh_.advertise<laser_line_extraction::LineSegmentList>("line_segments", 1);
  scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &LineExtractionROS::laserScanCallback, this);
  if (pub_markers_)
  {
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 1);
  }
}

LineExtractionROS::~LineExtractionROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::run()
{
  // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // Populate message
  laser_line_extraction::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);
  
  // Publish the lines
  line_publisher_.publish(msg);

  // Also publish markers if parameter publish_markers is set to true
  if (pub_markers_)
  {
    visualization_msgs::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
    marker_publisher_.publish(marker_msg);
  }

  findTargetFromLine(lines);

}
///////////////////////////////////////////////////////////////////////////////
// find goal station and publish tf
///////////////////////////////////////////////////////////////////////////////

void LineExtractionROS::findTargetFromLine(const std::vector<Line> &lines)
{
  // filter line with length, angle and distance
  std::vector<Line> lines_out;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    double radius = cit->getRadius(); // distance from origin to line
    double angle = cit->getAngle(); // rotate origin to orthogonal line
    double length = cit->length();

    // filter 1: length and angle
    // if(length<1.3 && length >0.7 && angle < -0.0 && angle > -1.5) // in laser frame
    if(length<3.0 && length >0.7 && angle < -0.0 && angle > -1.5) // in laser frame
    {
      lines_out.push_back(*cit);
    }

  }
  ROS_DEBUG("lines number after filter: %zu", lines_out.size());

  // calculate station tf 
  std::vector<double> closet_station;
  closet_station.resize(6); // left_x, left_y, right_x, right_y, angle, dist_to_centroid
  closet_station.clear();
  closet_station.push_back(0.0);
  closet_station.push_back(0.0);
  closet_station.push_back(0.0);
  closet_station.push_back(0.0);
  closet_station.push_back(0.0);
  closet_station.push_back(100.0);
  for (std::vector<Line>::const_iterator cit = lines_out.begin(); cit != lines_out.end(); ++cit)
  {
    boost::array<double, 2> right_tmp, left_tmp;
    right_tmp[0] = cit->getStart()[0]; 
    right_tmp[1] = cit->getStart()[1]; 
    left_tmp[0] = cit->getEnd()[0]; 
    left_tmp[1] = cit->getEnd()[1]; 
    if(cit->getStart()[1] > cit->getEnd()[1]) // set right points as start, already
    {
      boost::array<double, 2> points_tmp = right_tmp;
      right_tmp = left_tmp;
      left_tmp = points_tmp;
    }
    double line_angle = cit->getAngle();
    double line_angle2 = atan2(left_tmp[1]-right_tmp[1], left_tmp[0]-right_tmp[0]);

    // filter 2: in the front of vehicle
    // since the transform between laser and base_link is fixed, we transfer the point to base_link
    // tf base_link_to_laser: (0.275, 0.2, 0.2) (0, 0, 0.373425, 0.927660)
    double offset_x = 0.275;
    double offset_y = 0.2;
    double offset_angle = 0.765397; //43.85 degree, quat: (0, 0, 0.373425, 0.927660)

    double right_angle = atan2(right_tmp[1], right_tmp[0]);
    double left_angle = atan2(left_tmp[1], left_tmp[0]);
    double right_corner_bx = sqrt(right_tmp[0]*right_tmp[0]+right_tmp[1]*right_tmp[1]) * cos(offset_angle+right_angle);// + offset_x ; 
    double right_corner_by = sqrt(right_tmp[0]*right_tmp[0]+right_tmp[1]*right_tmp[1]) * sin(offset_angle+right_angle);// + offset_y ; 
    double left_corner_bx = sqrt(left_tmp[0]*left_tmp[0]+left_tmp[1]*left_tmp[1]) * cos(offset_angle+left_angle);// + offset_x ; 
    double left_corner_by = sqrt(left_tmp[0]*left_tmp[0]+left_tmp[1]*left_tmp[1]) * sin(offset_angle+left_angle);// + offset_y ; 
    ROS_DEBUG("line leftbefore : %4f %4f, right: %4f %4f, angle: %4f", 
        left_tmp[0], left_tmp[1], right_tmp[0], right_tmp[1], right_angle);

    ROS_DEBUG("line left: %4f %4f, right: %4f %4f, angle: %4f", 
        left_corner_bx, left_corner_by, right_corner_bx, right_corner_by, line_angle2);
    // ROS_DEBUG("line_angle: %f %f", line_angle, line_angle2);
    // ROS_DEBUG("left_tmp: %f %f", left_tmp[0], left_tmp[1]);
    // ROS_DEBUG("right_tmp: %f %f", right_tmp[0], right_tmp[1]);
    // ROS_DEBUG("line_param line_param: %f %f", line_angle, cit->getRadius());
    // ROS_DEBUG("rightleft: %f %f %f %f", right_tmp[0], right_tmp[1], left_tmp[0], left_tmp[1]);

    double left_x = -0.250*cos(line_angle2) + left_tmp[0]; // 0.270
    double left_y = -0.250*sin(line_angle2) + left_tmp[1];  // 0.270
    double right_x = 0.235*cos(line_angle2) + right_tmp[0]; // 0.235
    double right_y = 0.235*sin(line_angle2) + right_tmp[1]; // 0.235
    double center_x = (left_tmp[0]+left_tmp[1])/2.0;
    double center_y = (right_tmp[0]+right_tmp[1])/2.0;
    double dist = std::pow(center_x,2)+std::pow(center_x,2);
    // ROS_DEBUG("line left: %4f %4f, right: %4f %4f, angle: %4f, dist: %4f", 
    //     left_tmp[0], left_tmp[1], right_tmp[0], right_tmp[1], line_angle2, dist);
    // not the distance, but an Acute triangle, not Obtuse Triangle, continue ...  
    // if(dist < closet_station[5])
    if( (left_corner_by<0 && right_corner_by>0) || (left_corner_by>0 && right_corner_by<0) )
    {
      closet_station.clear();
      closet_station.push_back(left_x);
      closet_station.push_back(left_y);
      closet_station.push_back(right_x);
      closet_station.push_back(right_y);
      closet_station.push_back(line_angle2);
      closet_station.push_back(dist);
    }

  }
  // ROS_DEBUG("line left: %4f %4f, right: %4f %4f, angle: %4f, dist: %4f", 
  //     closet_station[0], closet_station[1], closet_station[2], closet_station[3], closet_station[4], closet_station[5]);
  
  // use old line param to filter noise
  if(line_for_filter.size()<10)
  {
    line_for_filter.push_back(closet_station);
  }
  else
  {
    line_for_filter.erase(line_for_filter.begin());
    line_for_filter.push_back(closet_station);
  }  


  // sum and average
  std::vector<double> line_filter_sum(6, 0.0);
  for (size_t line_id = 0; line_id < line_for_filter.size(); line_id++)
  {
    std::vector<double> line_tmp = line_for_filter[line_id];
    // ROS_DEBUG("line temp left: %4f %4f, right: %4f %4f, angle: %4f", 
        // line_tmp[0], line_tmp[1], line_tmp[2], line_tmp[3], line_tmp[4]);
    for (size_t kk = 0; kk < 6; kk++)
    {
      line_filter_sum[kk] +=  line_tmp[kk];
    }
  }

  // update the closet station
  if(line_for_filter.size()>0)
  {
    for (size_t kk = 0; kk < 6; kk++)
    {
      closet_station[kk] =  line_filter_sum[kk]/line_for_filter.size();
    }
    ROS_DEBUG("filter left: %4f %4f, right: %4f %4f, angle: %4f, dist: %4f", 
        closet_station[0], closet_station[1], closet_station[2], closet_station[3], closet_station[4], closet_station[5]);
  }
  else
  {
    ROS_DEBUG("not enough lines");
  }


  // publish only one station
  if(lines_out.size()>0 && closet_station[3]!=100)
  {
    std::string pub_left_link = "station_left";
    std::string pub_right_link = "station_right";
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(closet_station[0], closet_station[1], 0.0) );
    tf::Quaternion q;
    closet_station[4] = closet_station[4] - M_PI_2;
    q.setRPY(0, 0, closet_station[4]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "front_laser", pub_left_link));
    
    transform.setOrigin( tf::Vector3(closet_station[2], closet_station[3], 0.0) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "front_laser", pub_right_link));
  }

}

void LineExtractionROS::transformLinesToBaselink(const std::vector<Line> &lines_in,
                                                  std::vector<Line> &lines_out)
{
  lines_out.clear();
  for (std::vector<Line>::const_iterator cit = lines_in.begin(); cit != lines_in.end(); ++cit)
  {
    tf::Vector3 pts_sta_in(cit->getStart()[0], cit->getStart()[1], 0.0); 
    tf::Vector3 pts_end_in(cit->getEnd()[0], cit->getEnd()[1], 0.0); 
    // current rotation matrix
    tf::Matrix3x3 current_basis = transform_.getBasis();
    tf::Vector3 current_origin = transform_.getOrigin();

    tf::Vector3 pts_sta_tmp = current_basis*pts_sta_in+current_origin;
    tf::Vector3 pts_left_tmp = current_basis*pts_end_in+current_origin;
    tf::Vector3 pts_sta_final = pts_sta_tmp/pts_sta_tmp.z();
    tf::Vector3 pts_end_final = pts_left_tmp/pts_left_tmp.z();
    ROS_DEBUG("pts_origin: %f, %f, %f", pts_sta_in.x(), pts_sta_in.y(), pts_sta_in.z());
    ROS_DEBUG("pts_final: %f, %f, %f", pts_sta_final.x(), pts_sta_final.y(), pts_sta_final.z());
    boost::array<double, 2> right_tmp, left_tmp;
    right_tmp[0] = pts_sta_final.x(); 
    right_tmp[1] = pts_sta_final.y(); 
    left_tmp[0] = pts_end_final.x(); 
    left_tmp[1] = pts_end_final.y(); 
    // line_new = Line(cit->getAngle(), cit->getRadius(), cit->getCovariance(),
    //   right_tmp, left_tmp, cit->getIndices());
    Line line_new(cit->getAngle(), cit->getRadius(), cit->getCovariance(),
      right_tmp, left_tmp, cit->getIndices());
    lines_out.push_back(line_new);
    // ROS_DEBUG("line_odd: %f, %f", pts_origin.x(), pts_origin.y());
    // ROS_DEBUG("line_new: %f, %f", pts_final.x(), pts_final.y());
  }
}


///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{
  
  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node
  
  std::string frame_id, scan_topic;
  bool pub_markers;

  nh_local_.param<std::string>("frame_id", frame_id, "laser");
  frame_id_ = frame_id;
  ROS_DEBUG("frame_id: %s", frame_id_.c_str());

  nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
  scan_topic_ = scan_topic;
  ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

  nh_local_.param<bool>("publish_markers", pub_markers, false);
  pub_markers_ = pub_markers;
  ROS_DEBUG("publish_markers: %s", pub_markers ? "true" : "false");

  // Parameters used by the line extraction algorithm

  double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
         max_line_gap, min_line_length, min_range, max_range, min_split_dist, outlier_dist;
  int min_line_points;

  nh_local_.param<double>("bearing_std_dev", bearing_std_dev, 1e-3);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

  nh_local_.param<double>("range_std_dev", range_std_dev, 0.02);
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
  ROS_DEBUG("range_std_dev: %f", range_std_dev);

  nh_local_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);
  
  nh_local_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

  nh_local_.param<double>("max_line_gap", max_line_gap, 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);
  ROS_DEBUG("max_line_gap: %f", max_line_gap);

  nh_local_.param<double>("min_line_length", min_line_length, 0.5);
  line_extraction_.setMinLineLength(min_line_length);
  ROS_DEBUG("min_line_length: %f", min_line_length);

  nh_local_.param<double>("min_range", min_range, 0.4);
  line_extraction_.setMinRange(min_range);
  ROS_DEBUG("min_range: %f", min_range);

  nh_local_.param<double>("max_range", max_range, 10000.0);
  line_extraction_.setMaxRange(max_range);
  ROS_DEBUG("max_range: %f", max_range);

  nh_local_.param<double>("min_split_dist", min_split_dist, 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);
  ROS_DEBUG("min_split_dist: %f", min_split_dist);

  nh_local_.param<double>("outlier_dist", outlier_dist, 0.05);
  line_extraction_.setOutlierDist(outlier_dist);
  ROS_DEBUG("outlier_dist: %f", outlier_dist);

  nh_local_.param<int>("min_line_points", min_line_points, 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  ROS_DEBUG("min_line_points: %d", min_line_points);

  ROS_DEBUG("*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(const std::vector<Line> &lines,
                                                laser_line_extraction::LineSegmentList &line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    laser_line_extraction::LineSegment line_msg;
    line_msg.angle = cit->getAngle(); 
    line_msg.radius = cit->getRadius(); 
    line_msg.length = cit->length();
    line_msg.covariance = cit->getCovariance(); 
    line_msg.start = cit->getStart(); 
    line_msg.end = cit->getEnd(); 
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = ros::Time::now();
}

void LineExtractionROS::populateMarkerMsg(const std::vector<Line> &lines, 
                                           visualization_msgs::Marker &marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = ros::Time::now();
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg); 
    data_cached_ = true;
  }

  // if(!tf_cached_) // get transform at the beginning. 
  // {
  //   tf::StampedTransform transform;
  //   try{
  //     listener_.lookupTransform( "/base_link", scan_msg->header.frame_id,
  //                               ros::Time(0), transform);
  //     transform_ = transform;
  //     ROS_DEBUG("tf_transfrom cached: %f %f %f", transform_.getOrigin().x(),
  //                           transform_.getOrigin().y(), transform_.getOrigin().z());    
  //     tf_cached_ = true;
  //   }
  //   catch (tf::TransformException ex){
  //     ROS_ERROR("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //   }
  // }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

} // namespace line_extraction

