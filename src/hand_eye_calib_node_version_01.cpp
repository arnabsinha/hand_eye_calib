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

#include <url3Msg/url3Msg.h>
#include <Eigen/Dense>

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


class hand_eye_calib{
	private:
		ros::NodeHandle nh;
       typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//        typedef sensor_msgs::PointCloud<pcl::PointXYZ> PointCloud;
        typedef url3Msg::url3Msg robotMsgType;
		message_filters::Subscriber<Image> *image_sub, *depth_sub;
//		message_filters::Subscriber<PointCloud> *pcl_sub;
//        message_filters::Subscriber<robotMsgType> *robot_sub;
//		typedef sync_policies::ApproximateTime<Image, Image, robotMsgType> MySyncPolicy;
		typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
		Synchronizer<MySyncPolicy> *sync;

        ros::Publisher pub_image_pcl, pub_robot_pcl;

        Mat cameraM, distC, projM;
        Eigen::Matrix4f T, ef2Origin, base2EF, phy2image, ScalingM, ScalingMInv;
        float scaleUnit;
        tf::Transform transform;
        tf::Transform base2EF_tr, ef2Origin_tr;
        PointCloud physical_points_subset_, image_points_subset_;

        bool isCalib;
        tf::TransformListener listener;
        tf::StampedTransform tr;

        string kinectFileName, robotFileName, Path;
//        void callback(const ImageConstPtr& image, const ImageConstPtr& depth, const robotMsgType::ConstPtr& robotMsg);
        void callback(const ImageConstPtr& image, const ImageConstPtr& depth);
        void calibrate(PointCloud physical_points_, PointCloud image_points_, Mat &image);
        void projectPts( PointCloud physical_points_, Mat &image, vector<Point2f> &depthBuf);
        void getPhyPclFromTF(PointCloud &physical_points_);
        
	public:
		hand_eye_calib();
		~hand_eye_calib();
};

hand_eye_calib::hand_eye_calib()
	:nh("~")
{
	image_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/rgb_rect/image",1);
	depth_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/depth_rect/image",1);
//    robot_sub = new message_filters::Subscriber<robotMsgType> (nh, "/robot/state",1);
//    pcl_sub = new message_filters::Subscriber<PointCloud> (nh,"/url3/calib_points",1);

//	sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *image_sub, *depth_sub, *robot_sub);
//  sync->registerCallback(boost::bind(&hand_eye_calib::callback, this, _1, _2, _3));

	sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *image_sub, *depth_sub);
	sync->registerCallback(boost::bind(&hand_eye_calib::callback, this, _1, _2));
    cameraM = Mat::zeros(3,3,CV_32F);
    string cameraFile = getParam<string> (nh, "camera_calib_file","/home/ir/kinect2WS/src/iai_kinect2/kinect2_bridge/data/507554242542/calib_color.yaml");
    FileStorage fs2(cameraFile.c_str(), FileStorage::READ);
    fs2["cameraMatrix"]>>cameraM;
    fs2["distortionCoefficients"]>>distC;
    fs2["projection"]>>projM;
    fs2.release();
//    projM = projM.inv(DECOMP_LU);
    isCalib = false;
    listener.waitForTransform("robot_base",  "end_effector", ros::Time::now(), ros::Duration(0));
    try{
        listener.lookupTransform("robot_base",  "end_effector", ros::Time(0), tr);
        ROS_INFO_STREAM(tr.getOrigin().x());
    }
    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    ef2Origin = Eigen::Matrix4f::Identity();
    ef2Origin(0,3) = 0.0832;
    ef2Origin(1,3) = 0.0112;
    scaleUnit = 1.0;
    ScalingM = Eigen::Matrix4f::Identity();
    ScalingM *= 1.0/scaleUnit;
    ScalingM(3,3) = 1.0;
    ScalingMInv = ScalingM.inverse();
//    ScalingM *= scaleUnit;
    ef2Origin = ScalingMInv * ef2Origin * ScalingM;
//    ef2Origin(0,3) *= scaleUnit;
//    ef2Origin(1,3) *= scaleUnit;
    ef2Origin_tr = tfFromEigen(ef2Origin);

    pub_image_pcl = nh.advertise<PointCloud>("/imagepcl",10);
    pub_robot_pcl = nh.advertise<PointCloud>("/robotpcl",10);
}

hand_eye_calib::~hand_eye_calib(){

}

//void hand_eye_calib::callback(const ImageConstPtr& image, const ImageConstPtr& depth, const robotMsgType::ConstPtr& robotMsg)
void hand_eye_calib::callback(const ImageConstPtr& image, const ImageConstPtr& depth)
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
//            drawChessboardCorners( view, boardSize, Mat(pointBuf), found );
            PointCloud image_points_, physical_points_;
            float max_depth = 0.0;
            for(size_t i=0;i<pointBuf.size();i++){
                Mat ptM(4,1,projM.type());
                ptM.at<float>(0) = pointBuf[i].x;
                ptM.at<float>(1) = pointBuf[i].y;
                pcl::PointXYZ pt;
                pt.x = pointBuf[i].x;
                pt.y = pointBuf[i].y;
#if 0
                float temp=0.0, max_temp=0.0;
                for(int k1=-3;k1<=3;k1++)
                for(int k2=-3;k2<=3;k2++){
                    temp += static_cast<float>(depth_ptr->image.at<unsigned char>(pointBuf[i].x+k1, pointBuf[i].y+k2))/49.0;
//                    if(temp>max_temp && temp!=255)
//                        max_temp = temp;
                }
                for(int k1=-3;k1<=3;k1++)
                for(int k2=-3;k2<=3;k2++){
                    max_temp += pow(static_cast<float>(depth_ptr->image.at<unsigned char>(pointBuf[i].x+k1, pointBuf[i].y+k2))-temp,2.0)/49.0;
                }
                cout<<"Point "<<i<<" "<<temp<<" "<<sqrt(max_temp)<<endl;
                pt.z = temp;
#endif
                pt.z = static_cast<float>(depth_ptr->image.at<unsigned char>(pointBuf[i].x, pointBuf[i].y));
                ptM.at<float>(2) = pt.z;
                ptM.at<float>(3) = 1.0;
                ptM = projM * ptM;
#if 0
                if(ptM.at<float>(3)>0.1){
                    pt.x = ptM.at<float>(0)/ptM.at<float>(3);
                    pt.y = ptM.at<float>(1)/ptM.at<float>(3);
                    pt.z = ptM.at<float>(2)/ptM.at<float>(3);
                }
#endif
                image_points_.push_back(pt);
//                pt.z = 0.0;
//                if(max_temp>max_depth)
//                    max_depth = max_temp;///(float)pointBuf.size();
            }
//            for(size_t i=0;i<pointBuf.size();i++)
//                if(fabs(image_points_.points[i].z-max_depth)>20)
//                    image_points_.points[i].z = max_depth;
            image_points_.header.frame_id = "/kinect2_rgb_frame";
            pub_image_pcl.publish(image_points_);
#if 0
            BOOST_FOREACH (const pcl::PointXYZ& pt, cloud_in->points){
                physical_points_.push_back(pt);
            }
#endif
            getPhyPclFromTF(physical_points_);
            physical_points_.header.frame_id = "/robot_base";
            pub_robot_pcl.publish(physical_points_);
            ROS_INFO_STREAM(physical_points_.points.size()<<" "<<image_points_.points.size());
            
            if(!isCalib){
                calibrate(physical_points_, image_points_, view);
                vector<Point2f> projected_pts;
                projectPts(physical_points_, view, projected_pts);
#if 0
                cout<<"Printing original image points\n";
                for(size_t i=0;i<image_points_.points.size();i++)
                    cout<<image_points_.points[i].x<<" "<<image_points_.points[i].y<<" "<<image_points_.points[i].z<<endl;
#endif
                double dist = 0.0;
                for(size_t k=0;k<pointBuf.size();k++)
                    dist += sqrt(pow(pointBuf[k].x - projected_pts[k].x,2) + pow(pointBuf[k].y - projected_pts[k].y,2));
                ROS_INFO_STREAM("Calibration error "<<dist/(double)pointBuf.size());
                if(dist/(double)pointBuf.size()<20)
                    isCalib = true;
            }
#if 1
            else{
                vector<Point2f> projected_pts;
                projectPts(physical_points_, view, projected_pts);
                double dist = 0.0;
                for(size_t k=0;k<pointBuf.size();k++)
                    dist += sqrt(pow(pointBuf[k].x - projected_pts[k].x,2) + pow(pointBuf[k].y - projected_pts[k].y,2));
                ROS_INFO_STREAM("Calibration error "<<dist/(double)pointBuf.size());
                if(dist/(double)pointBuf.size()>20)
                    isCalib = false;
            }
#endif
#if 1
            imshow("result",view);
            waitKey(1);
#endif
        }

        ROS_INFO_STREAM("going for the next");
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void hand_eye_calib::getPhyPclFromTF(PointCloud &physical_points_){
    physical_points_.points.clear();
//    listener.waitForTransform("robot_base",  "end_effector", ros::Time::now(), ros::Duration(0));
    try{
        listener.lookupTransform("robot_base",  "end_effector", ros::Time(0), tr);
        ROS_INFO_STREAM(tr.getOrigin().x());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    base2EF_tr.setOrigin(tr.getOrigin());
    base2EF_tr.setRotation(tr.getRotation());
    cout<<tr.getOrigin().getX()<<" "<<tr.getOrigin().getY()<<" "<<tr.getOrigin().getZ()<<" "<<endl;
    base2EF = EigenFromTF(base2EF_tr);
//    base2EF(0,3) *= scaleUnit;
//    base2EF(1,3) *= scaleUnit;
//    base2EF(2,3) *= scaleUnit;
    base2EF = ScalingMInv * base2EF * ScalingM;
#if 0
    for(int i=0;i<3;i++)
        base2EF(i,3) = base2EF(i,3)*1000;
#endif
    
    T =  base2EF.inverse() * ef2Origin.inverse();
//    T =  ef2Origin * base2EF;
//    T(0,3) = T(0,3)*1000;
//    T(1,3) = T(1,3)*1000;
//    T(2,3) = T(2,3)*1000;
    
//    cout<<T<<endl;
//    T = T.inverse();
    cout<<"Base to End effector\n"<<T<<endl;
    for(int i=1;i<6;i++)
    for(int j=3;j>=-3;j--){
        Eigen::Vector4f Pt((float)i*0.0300*scaleUnit,(float)j*0.0300*scaleUnit,0.0,1.0);
        Eigen::Vector4f Tpt = T * Pt;
        Tpt = Tpt / Tpt(3);
        pcl::PointXYZ pt;
        pt.getVector4fMap() = Tpt;
        physical_points_.push_back(pt);
//        cout<<i<<" "<<j<<" "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
    }
}

void hand_eye_calib::calibrate(PointCloud physical_points_, PointCloud image_points_, Mat &image){
    physical_points_subset_.points.clear();    
    image_points_subset_.points.clear();    
    pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
    vector<int> pos;
#if 1
    pos.push_back(1);
    pos.push_back(3);
    pos.push_back(7);
    pos.push_back(30);
    pos.push_back(34);
    pos.push_back(5);
#endif
#if 0
    for(size_t i=1;i<physical_points_.points.size();i++)
        pos.push_back(i);
#endif
    vector<Point2f> i_pts, p_pts;
    Point2f mean_p;
    float scale = 0.0;
    for(int i=0;i<pos.size();i++){
        physical_points_subset_.push_back(physical_points_.points[pos[i]]);
        Point2f ptp;
        ptp.x = physical_points_.points[pos[i]].x;
        ptp.y = physical_points_.points[pos[i]].y;
        p_pts.push_back(ptp);
        mean_p += ptp;
        image_points_subset_.push_back(image_points_.points[pos[i]]);
        Point2f pti;
        pti.x = image_points_.points[pos[i]].x;
        pti.y = image_points_.points[pos[i]].y;
        i_pts.push_back(pti);
//        cout<<i<<" "<<ptp<<" "<<pti<<endl;
    }
#if 0
    mean_p.x /= (float)pos.size();
    mean_p.y /= (float)pos.size();
    for(int i=0;i<pos.size();i++){
        p_pts[i] -= mean_p;
        float temp = fabs(p_pts[i].x);// + fabs(p_pts[i].y);
        if(scale<temp)
            scale = temp;
        temp = fabs(p_pts[i].y);
        if(scale<temp)
            scale = temp;
//        cout<<scale<<" "<<p_pts[i].x<<" "<<p_pts[i].y<<endl;
    }
//    cout<<scale<<endl;
    scale = 20.0/scale;
//    cout<<scale<<endl;
    for(int i=0;i<pos.size();i++){
        p_pts[i].x = p_pts[i].x * scale;// + (float)image.rows/3.0;
        p_pts[i].y = p_pts[i].y * scale;// + (float)image.rows/3.0;
//        cout<<p_pts[i]<<" ";
    }
//    cout<<endl<<image.rows<<" "<<image.cols<<endl;
    mean_p.x = (float)image.rows/2.0;
    mean_p.y = (float)image.rows/2.0;
    for(int i=0;i<pos.size();i++){
        p_pts[i] += mean_p;
//        cout<<p_pts[i]<<" ";
//        circle(image, p_pts[i], 2, Scalar(0,0,255), 1, 8);
//        circle(image, i_pts[i], (i+4)*2, Scalar(255,0,0), 4, 8);
    }
//    cout<<endl;
#endif
    svd_estimator.estimateRigidTransformation(physical_points_subset_, image_points_subset_, phy2image);
    
    transform = tfFromEigen(phy2image);
    cout<<"Calibration result\n"<<phy2image<<endl;
}

void hand_eye_calib::projectPts( PointCloud physical_points_, Mat &image, vector<Point2f> &depthBuf){
    PointCloud trPcl;//, imagePcl;
//    Eigen::Matrix4f Tinv = base2EF * ef2Origin; //T.inverse();
    cout<<"Calibration stored\n"<<phy2image<<endl;
    transformPointCloud(physical_points_, trPcl, phy2image);
//    vector<Point2f> depthBuf;
//    cout<<"Printing transformed physical points\n";
    for(size_t i=0;i<trPcl.points.size();i++){
        Point2f pt;
        pt.x = trPcl.points[i].x;///trPcl.points[i].z;
        pt.y = trPcl.points[i].y;///trPcl.points[i].z;
        depthBuf.push_back(pt);
        circle(image, pt, 10, Scalar(0,0,255), 4, 8);
//        cout<<trPcl.points[i].x<<" "<<trPcl.points[i].y<<" "<<trPcl.points[i].z<<endl;
    }
    #if 0
    Size boardSize(7,5);
    drawChessboardCorners( image, boardSize, Mat(depthBuf), true );
    #endif
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hand_eye_calib_node");
	hand_eye_calib VC;
	ros::spin();

	return 0;
}


