#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "conio.h"
#include <iostream>
#include <fstream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;
using namespace Eigen;

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  tf::Matrix3x3 btm;
  btm.setValue(trans(0, 0), trans(0, 1), trans(0, 2),
               trans(1, 0), trans(1, 1), trans(1, 2),
               trans(2, 0), trans(2, 1), trans(2, 2));
  tf::Transform ret;
  ret.setOrigin(tf::Vector3(trans(0, 3), trans(1, 3), trans(2, 3)));
  ret.setBasis(btm);
  return ret;
}
Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  tf::Quaternion quat = trans.getRotation();
  tf::Vector3 origin = trans.getOrigin();

  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());

  out.topLeftCorner<3, 3>() = quat_out.toRotationMatrix();
  out.topRightCorner<3, 1>() = origin_out;
  out(3, 3) = 1;

  return out;
}

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
        T v;
        if (n.getParam(name, v))
        {
                ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
                return v;
        }
        else
                ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
        return defaultValue;
}

void CallBackFunc(int event, int x, int y);

class hand_eye_calib{
	private:
		ros::NodeHandle nh;
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		message_filters::Subscriber<Image> *image_sub, *depth_sub;
		message_filters::Subscriber<PointCloud> *pcl_sub;
		typedef sync_policies::ApproximateTime<Image, Image, PointCloud> MySyncPolicy;
		Synchronizer<MySyncPolicy> *sync;

        Mat cameraM, distC;
        Eigen::Matrix4f t;
        tf::Transform transform;

        bool isCalib;

        string kinectFileName, robotFileName, Path;
        void callback(const ImageConstPtr& image, const ImageConstPtr& depth, const PointCloud::ConstPtr& cloud_in );
        void calibrate(PointCloud physical_points_, PointCloud image_points_);
        void projectPts( PointCloud physical_points_, Mat &image);
        
	public:
		hand_eye_calib();
		~hand_eye_calib();
};

hand_eye_calib::hand_eye_calib()
	:nh("~")
{
	image_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/rgb_rect/image",1);
	depth_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/depth_rect/image",1);
    pcl_sub = new message_filters::Subscriber<PointCloud> (nh,"/url3/calib_points",1);
	sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *image_sub, *depth_sub, *pcl_sub);
	sync->registerCallback(boost::bind(&hand_eye_calib::callback, this, _1, _2, _3));

    cameraM = Mat::zeros(3,3,CV_32F);
    string cameraFile = getParam<string> (nh, "camera_calib_file","/home/ir/kinect2WS/src/iai_kinect2/kinect2_bridge/data/507554242542/calib_color.yaml");
    FileStorage fs2(cameraFile.c_str(), FileStorage::READ);
    fs2["cameraMatrix"]>>cameraM;
    fs2["distortionCoefficients"]>>distC;
    fs2.release();

    isCalib = false;
}

hand_eye_calib::~hand_eye_calib(){

}

void hand_eye_calib::callback(const ImageConstPtr& image, const ImageConstPtr& depth, const PointCloud::ConstPtr& cloud_in )
{
  // Solve all of perception here...
	ROS_INFO_STREAM("COOL");
    cv_bridge::CvImagePtr cv_ptr, depth_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
        vector<Point2f> pointBuf;
        Size boardSize(7,5);
        Mat view = cv_ptr->image.clone();
        bool found = findChessboardCorners( view, boardSize, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
            Mat viewGray;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
            drawChessboardCorners( view, boardSize, Mat(pointBuf), found );
            PointCloud image_points_, physical_points_;
            for(size_t i=0;i<pointBuf.size();i++){
                pcl::PointXYZ pt;
                pt.x = pointBuf[i].x;
                pt.y = pointBuf[i].y;
                pt.z = static_cast<float>(depth_ptr->image.at<unsigned char>(pointBuf[i].x, pointBuf[i].y));
                image_points_.push_back(pt);
            }
            BOOST_FOREACH (const pcl::PointXYZ& pt, cloud_in->points){
                physical_points_.push_back(pt);
            }
            if(!isCalib)
                calibrate(physical_points_, image_points_);
            else
                projectPts(physical_points_, view);
            imshow("result",view);
            waitKey(1);
        }

        ROS_INFO_STREAM("going for the next");
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void hand_eye_calib::calibrate(PointCloud physical_points_, PointCloud image_points_){
    pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
    svd_estimator.estimateRigidTransformation(physical_points_, image_points_, t);
    transform = tfFromEigen(t);
}

void hand_eye_calib::projectPts( PointCloud physical_points_, Mat &image){
    PointCloud trPcl;
    transformPointCloud(physical_points_, trPcl, t);
    vector<Point2f> depthBuf;
    for(size_t i=0;i<trPcl.points.size();i++){
        Point2f pt;
        pt.x = trPcl.points[i].x;
        pt.y = trPcl.points[i].y;
        depthBuf.push_back(pt);
    }
    Size boardSize(7,5);
    drawChessboardCorners( image, boardSize, Mat(depthBuf), true );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hand_eye_calib_node");
	hand_eye_calib VC;
	ros::spin();

	return 0;
}


