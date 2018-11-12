//
// Created by rakesh on 27/08/18.
//

#include <icp_slam/mapper.h>

namespace icp_slam
{
// 

void Mapper::initMap(int width, int height, float resolution,double origin_x_meters, double origin_y_meters,
               uint8_t *pointer=nullptr, unsigned char unknown_cost_value=NO_INFORMATION)
{
	//TODO: intialize the map
  

  width_=this.width;
  height_=this.height;
  resolution_=this.resolution;  ///< @brief meters/pixels

  map_ = cv::Mat(height/resolution,width/resolution,int);

  origin_x_=this.origin_x_meters ; ///< origin of the map in meters
  origin_y_=this.origin_y_meters; ///< origin of the map in meters

  init_robot_pose_.x=0;
  init_robot_pose_.y=0;
  init_robot_pose_.a=0;

  init_robot_pt_x =(width/(2*resolution));
  init_robot_pt_y =(height/(2*resolution));

  for(i=0;i<height/resolution;i++)
  {
  	for(j=0;j<width/resolution;j++)
  	{
  		map_.at<int>(i,j)=(int)NO_INFORMATION;
  	}
  }

  is_initialized_ = true;

  relative_map_ = cv::Mat::zeros(height/resolution,width/resolution,int);

}


robot_pose_t Mapper::getRobotPose(tf::StampedTransform &tf_map_laser)
{
	robot_pose_.x=(double)tf_map_laser.getOrigin().getX();
	robot_pose_.y=(double)tf_map_laser.getOrigin().getY();
	robot_pose_.a=(double)tf_map_laser.tf::getYaw(last_kf_tf.getRotation());

	return robot_pose_;
}

cv::point Mapper::getRobotpoints(robot_pose_t &init_pose,
							robot_pose_t &current_pose,
							double &point_x,
							double &point_y)
{
	cv::Point2d robot;
	point_x=(double)(current_pose.x+init_pose.x);
	point_y=(double)(current_pose.y+init_pose.x);
	robot(point_x,point_y);

	return | 


}

} // namespace icp_slam
