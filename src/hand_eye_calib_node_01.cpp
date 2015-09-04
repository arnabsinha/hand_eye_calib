#include <ros/ros.h>

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
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>

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

class hand_eye_calib{
	private:
		ros::NodeHandle nh;
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		message_filters::Subscriber<Image> *image_sub, *depth_sub;
		typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
		Synchronizer<MySyncPolicy> *sync;

        ros::Publisher pub_image_pcl, pub_robot_pcl;

        Mat cameraM, distC, projM;
        Eigen::Matrix4f T, ef2Origin, base2EF, phy2image, ScalingM, ScalingMInv, phy2imageEarly;
        float scaleUnit;
        tf::Transform transform;
        tf::Transform base2EF_tr, ef2Origin_tr;
        PointCloud physical_points_subset_, image_points_subset_;

        bool isCalib;
        tf::TransformListener listener;
        tf::StampedTransform tr;
        double cx, cy, fx, fy, actCalErr;

        string kinectFileName, robotFileName, Path;
        void callback(const ImageConstPtr& image, const ImageConstPtr& depth);
        void calibrate(PointCloud physical_points_, PointCloud image_points_, Mat &image);
        void projectPts( PointCloud physical_points_, Mat &image, vector<Point2f> &depthBuf, vector<Point2f> pointBuf);
        void getPhyPclFromTF(PointCloud &physical_points_);
        void transform_polar_to_cart(pcl::PointXYZ pti, pcl::PointXYZ &pto);
        void transform_cart_to_polar(pcl::PointXYZ pti, pcl::PointXYZ &pto);
        
	public:
		hand_eye_calib();
		~hand_eye_calib();
};


hand_eye_calib::hand_eye_calib()
	:nh("~")
{
	image_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/hd/image_color_rect",1);
	depth_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/hd/image_depth_rect",1);

	sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *image_sub, *depth_sub);
	sync->registerCallback(boost::bind(&hand_eye_calib::callback, this, _1, _2));
    cameraM = Mat::zeros(3,3,CV_32F);
    string cameraFile = getParam<string> (nh, "camera_calib_file","/home/ir/kinect2WS/src/iai_kinect2/kinect2_bridge/data/507554242542/calib_color.yaml");
    FileStorage fs2(cameraFile.c_str(), FileStorage::READ);
    fs2["cameraMatrix"]>>cameraM;
    fs2["distortionCoefficients"]>>distC;
    fs2["projection"]>>projM;
    fs2.release();
    fx = cameraM.at<double>(0,0);
    fy = cameraM.at<double>(1,1);
    cx = cameraM.at<double>(0,2);
    cy = cameraM.at<double>(1,2);
//    projM = projM.inv(DECOMP_LU);
    phy2imageEarly = Eigen::Matrix4f::Identity();
    isCalib = getParam<bool> (nh,"calibrated",false);
    if(isCalib){
        ifstream fpCalib;
        fpCalib.open("/home/ir/perception_ws/src/hand_eye_calib/src/hand_eye_calib.txt");
        double temp=0.0;
        for(size_t i=0;i<4;i++)
        for(size_t j=0;j<4;j++){
            fpCalib>>temp;
            phy2imageEarly(i,j) = temp;
        }
    }
    cout<<phy2imageEarly<<endl;
    listener.waitForTransform("robot_base",  "end_effector", ros::Time::now(), ros::Duration(0));
    try{
        listener.lookupTransform("robot_base",  "end_effector", ros::Time(0), tr);
        //ROS_INFO_STREAM(tr.getOrigin().x());
    }
    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    ef2Origin = Eigen::Matrix4f::Identity();
    ef2Origin(2,3) = 0.0832;
    ef2Origin(1,3) = -0.0112;
    scaleUnit =100.0;//00.0;//100.0;
    ScalingM = Eigen::Matrix4f::Identity();
    ScalingM *= scaleUnit;
    ScalingM(3,3) = 1.0;
    ScalingMInv = ScalingM.inverse();
    ef2Origin = ScalingM * ef2Origin * ScalingMInv;
    ef2Origin_tr = tfFromEigen(ef2Origin);

    pub_image_pcl = nh.advertise<PointCloud>("/imagepcl",10);
    pub_robot_pcl = nh.advertise<PointCloud>("/robotpcl",10);
}

hand_eye_calib::~hand_eye_calib(){

}

void hand_eye_calib::callback(const ImageConstPtr& image, const ImageConstPtr& depth)
{
  // Solve all of perception here...
	//ROS_INFO_STREAM("COOL");
    cv_bridge::CvImagePtr cv_ptr, depth_ptr;
    try
    {
        Mat tempMat;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
//        tempMat = cv_ptr->image.clone();
        depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
//        undistort(tempMat, cv_ptr->image, cameraM, distC);
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
            ROS_INFO_STREAM("Detected the pattern "<<pointBuf.size());
            for(size_t i=0;i<pointBuf.size();i++){
                pcl::PointXYZ pt;
                pt.x = pointBuf[i].x;///(float)cameraM.at<double>(0,0);
                pt.y = pointBuf[i].y;///(float)cameraM.at<double>(1,1);
                double temp, max_temp = 0.0;
                for (int i1=-2;i1<=2;i1++)
                for (int j1=-2;j1<=2;j1++){
                    temp = static_cast<double>(depth_ptr->image.at<unsigned char>((pointBuf[i].x+i1, pointBuf[i].y+j1));// - 57.4745;
                    if(temp>max_temp)
                        max_temp = temp;
//                    cout<<"From here:"<<i<<" "<<i1<<" "<<j1<<" "<<temp<<" "<<static_cast<double>(depth_ptr->image.at<unsigned char>(pointBuf[i].x+i1, pointBuf[i].y+j1))<<" "<<max_temp<<endl;
                }
                pt.z = max_temp;
                image_points_.push_back(pt);
            }
            getPhyPclFromTF(physical_points_);
            physical_points_.header.frame_id = "/robot_base";
            pub_robot_pcl.publish(physical_points_);
            
#if 1
            //ROS_INFO_STREAM(physical_points_.points.size()<<" "<<image_points_.points.size());
            if(!isCalib){
                char kb = getche();
                if(kb==' '){
                    calibrate(physical_points_, image_points_, view);
                    vector<Point2f> projected_pts;
                    projectPts(physical_points_, view, projected_pts, pointBuf);
/*
                double dist = 0.0;
                for(size_t k=0;k<pointBuf.size();k++)
                    dist += sqrt(pow(pointBuf[k].x - projected_pts[k].x,2) + pow(pointBuf[k].y - projected_pts[k].y,2));
                ROS_INFO_STREAM("Calibration error "<<dist/(double)pointBuf.size());
                if(dist/(double)pointBuf.size()<20)
                    isCalib = true;
*/
                }
                else{
                    if(kb=='d'){
                        isCalib = true;
                        ofstream fpCalib;
                        fpCalib.open("/home/ir/perception_ws/src/hand_eye_calib/src/hand_eye_calib.txt");
                        fpCalib<<phy2image;
                        fpCalib.close();
                    }
                    phy2image = phy2imageEarly;
                    vector<Point2f> projected_pts;
                    projectPts(physical_points_, view, projected_pts, pointBuf);
                }
            }
            else{
                phy2image = phy2imageEarly;
                vector<Point2f> projected_pts;
                projectPts(physical_points_, view, projected_pts, pointBuf);
            }
            
#if 0
            else{
                vector<Point2f> projected_pts;
                projectPts(physical_points_, view, projected_pts, pointBuf);
                double dist = 0.0;
                for(size_t k=0;k<pointBuf.size();k++)
                    dist += sqrt(pow(pointBuf[k].x - projected_pts[k].x,2) + pow(pointBuf[k].y - projected_pts[k].y,2));
                ROS_INFO_STREAM("Calibration error "<<dist/(double)pointBuf.size());
                if(dist/(double)pointBuf.size()>20)
                    isCalib = false;
            }
#endif
            imshow("result",view);
            waitKey(1);
#endif

        }

        ROS_INFO_STREAM(endl<<phy2imageEarly);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void hand_eye_calib::getPhyPclFromTF(PointCloud &physical_points_){
    physical_points_.points.clear();
    try{
        listener.lookupTransform("robot_base",  "end_effector", ros::Time(0), tr);
        //ROS_INFO_STREAM(tr.getOrigin().x());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    base2EF_tr.setOrigin(tr.getOrigin());
    base2EF_tr.setRotation(tr.getRotation());
    //cout<<tr.getOrigin().getX()<<" "<<tr.getOrigin().getY()<<" "<<tr.getOrigin().getZ()<<" "<<endl;
    base2EF = EigenFromTF(base2EF_tr);
    base2EF = ScalingM * base2EF * ScalingMInv;
    
//    T =  base2EF.inverse() * ef2Origin.inverse();
//    T =   ef2Origin * base2EF ;
    T =   base2EF * ef2Origin;
//    T = base2EF * ef2Origin.inverse();
//    T = base2EF.inverse() * ef2Origin;
    //cout<<"Base to End effector\n"<<T<<endl;
    for(int j=1;j<6;j++)
    for(int i=3;i>=-3;i--){
        Eigen::Vector4f Pt((float)i*0.0300*scaleUnit,0.0,(float)j*0.0300*scaleUnit,1.0);
        Eigen::Vector4f Tpt = T * Pt;
        pcl::PointXYZ pt;
        if(fabs(Tpt(3))>0.1){
            Tpt = Tpt / Tpt(3);
            pt.getVector4fMap() = Tpt;
        }
        else{
            pt.z = 0.0;
        }
#if 0
        pt.x = pt.x * 100.0;
        pt.y = pt.y * 100.0;
        pt.z = pt.z * 100.0;
#endif
        physical_points_.push_back(pt);
    }
}

void hand_eye_calib::calibrate(PointCloud physical_points_, PointCloud image_points_, Mat &image){
//    physical_points_subset_.points.clear();    
//    image_points_subset_.points.clear();    
//    pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
    pcl::registration::TransformationEstimationDualQuaternion < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
//    pcl::registration::TransformationEstimationLM < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
    vector<int> pos;
    for(int i=0;i<physical_points_.points.size();i++)
        pos.push_back(i);
#if 0
    pos.push_back(1);
    pos.push_back(34);
    pos.push_back(3);
    pos.push_back(7);
    pos.push_back(20);
    pos.push_back(7);
    pos.push_back(12);
    pos.push_back(5);
#endif
    for(int i=0;i<pos.size();i++){
        pcl::PointXYZ pt;
        transform_polar_to_cart(image_points_.points[pos[i]], pt);
        if(pt.z>0 && fabs(physical_points_.points[pos[i]].z)>0.001){
            physical_points_subset_.push_back(physical_points_.points[pos[i]]);
            image_points_subset_.push_back(pt);
        }
    }
    image_points_subset_.header.frame_id = "/kinect2_rgb_optical_frame";
    pub_image_pcl.publish(image_points_subset_);
    if(physical_points_subset_.points.size()>physical_points_.points.size()){
        ROS_INFO_STREAM("Doing calibration");
#if 1
        for(size_t i=0;i<physical_points_subset_.points.size();i++){
            cout<<physical_points_subset_.points[i].x<<" "<<physical_points_subset_.points[i].y<<" "<<physical_points_subset_.points[i].z<<"\t"<<image_points_subset_.points[i].x<<" "<<image_points_subset_.points[i].y<<" "<<image_points_subset_.points[i].z<<" "<<endl;
        }
#endif
        svd_estimator.estimateRigidTransformation(physical_points_subset_, image_points_subset_, phy2image);
        PointCloud test;
        transformPointCloud(physical_points_subset_, test, phy2image);
        double testD = 0.0;
        for(size_t ii=0;ii<physical_points_subset_.points.size();ii++)
            testD += sqrt(pow(test.points[ii].x-image_points_subset_.points[ii].x,2.0) + pow(test.points[ii].y-image_points_subset_.points[ii].y,2.0) + pow(test.points[ii].z-image_points_subset_.points[ii].z,2.0))/(double)(physical_points_subset_.points.size());
        ROS_INFO_STREAM("Calibration ERROR "<<testD); 
        actCalErr = testD;
        
        transform = tfFromEigen(phy2image);
    }
    else
        ROS_INFO_STREAM("Number of points is "<<physical_points_subset_.points.size()<<" ; hence not doing calibration");
}

void hand_eye_calib::transform_polar_to_cart(pcl::PointXYZ pti, pcl::PointXYZ &pto){
#if 1
    pto.z = pti.z;
    pto.x = (pti.x - cx)*pto.z/fx;
    pto.y = (pti.y - cy)*pto.z/fy;
#endif
#if 0
    Mat ptIn = Mat::zeros(3,1,cameraM.type());
    Mat ptOut = ptIn.clone();
    ptIn.at<double>(0) = pti.x*pti.z;
    ptIn.at<double>(1) = pti.y*pti.z;
    ptIn.at<double>(2) = pti.z;
    Mat cameraMInv = Mat::zeros(cameraM.size(),cameraM.type());
    invert(cameraM, cameraMInv, DECOMP_LU);
    ptOut = cameraMInv * ptIn;
    cout<<pto.x<<" "<<ptOut.at<double>(0)<<" "<<pto.y<<" "<<ptOut.at<double>(1)<<" "<<pto.z<<" "<<ptOut.at<double>(2)<<" "<<endl;
    pto.x = ptOut.at<double>(0);
    pto.y = ptOut.at<double>(1);
    pto.z = ptOut.at<double>(2);
#endif
//    cout<<"From Image space to camera space:"<<pti.x<<" "<<pti.y<<" "<<pti.z<<"\t"<<pto.x<<" "<<pto.y<<" "<<pto.z<<endl;
}

void hand_eye_calib::transform_cart_to_polar(pcl::PointXYZ pti, pcl::PointXYZ &pto){
#if 0
    pto.z = pti.z;
    pto.x = pti.x*fx/pto.z + cx;
    pto.y = pti.y*fy/pto.z + cy;
#endif
#if 1
    Mat ptIn = Mat::zeros(3,1,cameraM.type());
    Mat ptOut = ptIn.clone();
    ptIn.at<double>(0) = pti.x;
    ptIn.at<double>(1) = pti.y;
    ptIn.at<double>(2) = pti.z;
    ptOut = cameraM * ptIn;
//    cout<<pto.x<<" "<<ptOut.at<double>(0)/ptOut.at<double>(2)<<" "<<pto.y<<" "<<ptOut.at<double>(1)/ptOut.at<double>(2)<<" "<<pto.z<<" "<<ptOut.at<double>(2)<<" "<<endl;
    pto.z = ptOut.at<double>(2);
    pto.x = ptOut.at<double>(0)/pto.z;
    pto.y = ptOut.at<double>(1)/pto.z;
    
#endif
//    cout<<"From camera space to image space:"<<pti.x<<" "<<pti.y<<" "<<pti.z<<"\t"<<pto.x<<" "<<pto.y<<" "<<pto.z<<endl;
}
void hand_eye_calib::projectPts( PointCloud physical_points_, Mat &image, vector<Point2f> &depthBuf, vector<Point2f> pointBuf){
    PointCloud trPcl, trPclEarly;//, imagePcl;
//    cout<<"Calibration stored\n"<<phy2image<<endl;
    transformPointCloud(physical_points_, trPcl, phy2image);
    transformPointCloud(physical_points_, trPclEarly, phy2imageEarly);
    vector<Point2f> earlyPts, nowPts;
    for(size_t i=0;i<trPcl.points.size();i++){
        pcl::PointXYZ pti, ptpi;
        transform_cart_to_polar(trPcl.points[i], pti);
        transform_cart_to_polar(trPclEarly.points[i], ptpi);
        Point2f pt, ptp;
        pt.x = pti.x;//*(float)cameraM.at<double>(0,0);///trPcl.points[i].z;
        pt.y = pti.y;//*(float)cameraM.at<double>(1,1);///trPcl.points[i].z;
        ptp.x = ptpi.x;//*(float)cameraM.at<double>(0,0);
        ptp.y = ptpi.y;//*(float)cameraM.at<double>(1,1);
        nowPts.push_back(pt);
        earlyPts.push_back(ptp);
//        circle(image, pt, 10, Scalar(0,0,255), 4, 8);
    }
    double dist = 0.0, distP = 0.0;
    for(size_t k=0;k<pointBuf.size();k++){
        dist += sqrt(pow(pointBuf[k].x - nowPts[k].x,2) + pow(pointBuf[k].y - nowPts[k].y,2));
        distP += sqrt(pow(pointBuf[k].x - earlyPts[k].x,2) + pow(pointBuf[k].y - earlyPts[k].y,2));
    }
    dist /= (float)pointBuf.size();
    distP /= (float)pointBuf.size();
    Size boardSize(7,5);
    ROS_INFO_STREAM("Image calibration error : "<<dist<<" "<<distP<<" "<<actCalErr);
    if(distP>dist){
        phy2imageEarly = phy2image;
        for(size_t k=0;k<pointBuf.size();k++){
            depthBuf.push_back(nowPts[k]);
//            circle(image, nowPts[k], 10, Scalar(0,0,255), 4, 8);
        }
    }
    else{
        for(size_t k=0;k<pointBuf.size();k++){
            depthBuf.push_back(earlyPts[k]);
//            circle(image, earlyPts[k], 10, Scalar(0,0,255), 4, 8);
        }
    }
    for(size_t i=0;i<depthBuf.size();i++){
        circle(image, depthBuf[i], 2+i/2, Scalar(i*5,0,255-i*5), 2, 8);
        circle(image, pointBuf[i], 2+i/2, Scalar(255-i*5,0,i*5), 2, 8);
    }
        
    //drawChessboardCorners( image, boardSize, Mat(depthBuf), true );
    //drawChessboardCorners( image, boardSize, Mat(pointBuf), true );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hand_eye_calib_node");
	hand_eye_calib VC;
	ros::spin();

	return 0;
}


