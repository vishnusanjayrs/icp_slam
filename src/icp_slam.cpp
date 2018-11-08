//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_pc_mat_(),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{


  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }
  // for the first time . set the first odom frame as keyframe and as well as map frame
  is_tracker_running_=true;
  if (keyframe_count_ == 0) 
  {
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = current_frame_tf_odom_laser;
    last_kf_pc_mat_ = utils::laserScanToPointMat(laser_scan);
    tf_map_laser = current_frame_tf_odom_laser;
    ROS_INFO("First keyframe");
    keyframe_count_++;
  }
  
  if (isCreateKeyframe(current_frame_tf_odom_laser,last_kf_tf_odom_laser_))
  {
    ROS_INFO("keyframe created");
    auto T_2_1 = current_frame_tf_odom_laser.inverse()*last_kf_tf_odom_laser_;
    auto curr_point_mat_=utils::laserScanToPointMat(laser_scan);
    tf::Transform map2laser=icpRegistration(last_kf_pc_mat_,curr_point_mat_,T_2_1);
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_pc_mat_ = curr_point_mat_;
  }

  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
  is_tracker_running_=false;
  return false;
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  auto current_x = current_frame_tf.getOrigin().getX();
  auto current_y = current_frame_tf.getOrigin().getY();
  auto current_rotation = tf::getYaw(current_frame_tf.getRotation()) * 180/M_PI;

  auto last_kf_x = last_kf_tf.getOrigin().getX();
  auto last_kf_y = last_kf_tf.getOrigin().getY();
  auto last_kf_rotation = tf::getYaw(last_kf_tf.getRotation()) * 180/M_PI;

  auto distance = sqrt(pow((current_x - last_kf_x),2)+pow((current_y - last_kf_y),2));
  auto rotation_diff = abs(last_kf_rotation - current_rotation);

  auto current_time = ros::Time::now().toSec();
  auto last_kf_time = last_kf_tf.stamp_.toSec();

  auto last_kf_age = current_time - last_kf_time ;

  
  if ((distance > max_keyframes_distance_) ||(rotation_diff > max_keyframes_angle_)||(last_kf_age > max_keyframes_time_))
  {
    ROS_INFO("robot pose: (%f, %f), %f",current_x,current_y, current_rotation);
    return true;
  }
  else
  {
    return false;
  }
}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1)
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite("/tmp/icp_laser.png", img);
}

tf::Transform ICPSlam::icpRegistration(const cv::Mat &last_point_mat,
                                              const cv::Mat &curr_point_mat,
                                              const tf::Transform &T_2_1)
{
  cv::Mat point_mat1_=last_point_mat;
  cv::Mat point_mat2_=curr_point_mat;
  cv::Mat inlier_mat1=cv::Mat(1,2,CV_32F);
  cv::Mat inlier_mat2=cv::Mat(1,2,CV_32F);
  cv::Mat inlier_xy_mat=cv::Mat(1,2,CV_32F);
  cv::Mat icp_mat1;
  cv::Mat icp_mat2;
  float mean;
  float std_dev;
  size_t vec_size;
  std::vector<int> closest_indices;
  std::vector<float> closest_distances_2;
  int max_iterations = 10;
  tfScalar last_x;
  tfScalar last_y;
  tfScalar last_r;
  tfScalar curr_x;
  tfScalar curr_y;
  tfScalar curr_t;
  float sum_1_x ;
  float sum_2_x ;
  float mean_1_x ;
  float mean_2_x ;
  float sum_1_y ;
  float sum_2_y ;
  float mean_1_y ; 
  float mean_2_y ;
  int match_idx;
  int i;
  float dist_thres;
  tf::Transform transformed_point_mat2_;
  cv::Mat x_mean=cv::Mat(2,1,CV_32F);
  cv::Mat p_mean=cv::Mat(2,1,CV_32F); 
  


  for (int iter=0;iter<max_iterations;i++)
  {
    sum_1_x = 0.0;
    sum_2_x = 0.0;
    mean_1_x = 0.0;
    mean_2_x = 0.0;
    sum_1_y = 0.0;
    sum_2_y = 0.0;
    mean_1_y = 0.0; 
    mean_2_y = 0.0;
    transformed_point_mat2_ = utils::transformPointMat(T_2_1,point_mat2_);
  
    // get the closest point correspondence
    ICPSlam::closestPoints(point_mat1_,transformed_point_mat2_,closest_indices,closest_distances_2);

    for(i=0;i<50;i++)
    {
      ROS_INFO("index: (%i) = %i",i,closest_indices[i]);
    }
    //vizClosestPoints(point_mat1_,point_mat2_,T_2_1);
  
    //get inliers
    utils::meanAndStdDev(closest_distances_2,mean,std_dev);
    vec_size = closest_indices.size();
    dist_thres = mean +(2*std_dev);
    match_idx=0;
    for(i=0;i<vec_size;i++)
    {
      ///distance less than threshold or when indices are valid or any one xs are not 0.0(special case less than range_min) are added to a new matrix
      if(closest_distances_2[i]<dist_thres || closest_indices[i] != -1 || point_mat1_[i][0]!=0.0||point_mat2_[closest_indices[i]]!=0.0)
      {
        if(match_idx==0)
        {
          inlier_mat1.at<float>(0,0) = point_mat1_[i][0];
          sum_1_x = sum_1_x+point_mat1_[i][0];
          
          inlier_mat1.at<float>(0,1) = point_mat1_[i][1];
          sum_1_y=sum_1_y+point_mat1_[i][1];


          inlier_mat2.at<float>(0,0) = point_mat2_[closest_indices[i]][0];
          sum_2_x=sum_2_x+point_mat2_[closest_indices[i]][0];
          
          inlier_mat2.at<float>(0,1) = point_mat2_[closest_indices[i]][1];
          sum_2_y=sum_2_y+point_mat2_[closest_indices[i]][1];

        }
        else
        {
          inlier_xy_mat.at<float>(0,0) = point_mat1_[i][0];
          sum_1_x = sum_1_x+point_mat1_[i][0];

          inlier_xy_mat.at<float>(0,1) = point_mat1_[i][1];
          sum_1_y=sum_1_y+point_mat1_[i][1];

          inlier_mat1.push_back(inlier_xy_mat);

          inlier_xy_mat.at<float>(0,0) = point_mat2_[closest_indices[i]][0];
          sum_2_x=sum_2_x+point_mat2_[closest_indices[i]][0];

          inlier_xy_mat.at<float>(0,1) = point_mat2_[closest_indices[i]][1];
          sum_2_y=sum_2_y+point_mat2_[closest_indices[i]][1];

          inlier_mat2.push_back(inlier_xy_mat);
        }
        
        match_idx++;
      }
    }
    mean_1_x=sum_1_x/(match_idx-1);
    mean_1_y=sum_1_y/(match_idx-1);
    mean_2_x=sum_2_x/(match_idx-1);
    mean_2_y=sum_2_y/(match_idx-1);
    icp_mat1=cv::Mat((match_idx-1),2,CV_32F);
    icp_mat2=cv::Mat((match_idx-1),2,CV_32F);
    for(i=0;i<match_idx;i++)
    {
      icp_mat1.at<float>(i,0)=inlier_mat1[i][0]-mean_1_x;
      icp_mat1.at<float>(i,1)=inlier_mat1[i][1]-mean_1_y;

      icp_mat2.at<float>(i,0)=inlier_mat1[i][0]-mean_2_x;
      icp_mat2.at<float>(i,1)=inlier_mat1[i][1]-mean_2_y;

    }


    auto last_T_2_1 = T_2_1;

    x_mean.at<float>(0,0)= mean_1_x;
    x_mean.at<float>(1,0)= mean_1_y;

    p_mean.at<float>(0,0)= mean_2_x;
    p_mean.at<float>(1,0)= mean_2_y;


    tf::transform T_2_1 = icpIteration(icp_mat1,icp_mat2,x_mean,p_mean);

    last_x = last_T_2_1.getOrigin().getX();
    last_y = last_T_2_1.getOrigin().getY();
    last_r = tf::getYaw(last_T_2_1.getRotation()) * 180/M_PI;

    curr_x = T_2_1.getOrigin().getX();
    curr_y = T_2_1.getOrigin().getY();
    curr_r = tf::getYaw(T_2_1.getRotation()) * 180/M_PI;

    if (last_x == curr_x && last_y =curr_y && last_r==curr_r)
    {
      return T_2_1;
    }
  }
  //transform the point_cloud2 to point_cloud1 using initial odom pose
return T_2_1;
}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2,
                                    cv::Mat &x_mean,
                                    cv::Mat &p_mean)
{
  cv::Mat W=cv::Mat::zeros(2,2,CV_32F);
  int nrows=point_mat2.rows;
  for (int i=0;i<nrows;i++)
  {
    W=W+((point_mat1.row(i).t())*(point_mat2.row(i)));
  }

  cv::SVD svd(W);

  auto R_matrix = svd.u*svd.vt;
  auto T_matrix = x_mean-(R*p_mean);

  auto rotation_angle = std::acos(R_matrix[0][0]);

  
  
  
}

} // namespace icp_slam

