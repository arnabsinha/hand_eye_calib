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

#include "conio.h"
#include <iostream>
#include <fstream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;
//TODO we do not need geometry message rather we need the vicon marker message
// TODO change the code accordingly
class vicon{
	private:
		ros::NodeHandle nh;
        typedef geometry_msgs::TransformStamped viconMsgType;
		message_filters::Subscriber<Image> *image_sub;
		message_filters::Subscriber<viconMsgType> *info_sub;
		typedef sync_policies::ApproximateTime<Image, viconMsgType> MySyncPolicy;
		Synchronizer<MySyncPolicy> *sync;

        bool calibrated;
        Mat R, T, cameraM, distC;
        void getPositions(vector<Point2f> &markerImg, viconMsgType viconMsg);

        string img_path, vicon_path;
        int num, num_mouse, max_mouse;
        
	public:
		void callback(const ImageConstPtr& image, const viconMsgType::ConstPtr& viconMsg);
		vicon();
		~vicon();
};

vicon::vicon()
	:nh("~")
{
	image_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/rgb_rect/image",1);
	info_sub = new message_filters::Subscriber<viconMsgType> (nh,"/vicon/astarTest/astarTest",1);
	sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *image_sub, *info_sub);
	sync->registerCallback(boost::bind(&vicon::callback, this, _1, _2));
    calibrated = false;
    img_path = "/home/ir/perception_ws/src/vicon_with_camera/data/image%d.png";
    vicon_path = "/home/ir/perception_ws/src/vicon_with_camera/data/vicon%d.txt";
    num = 0;

    cameraM = Mat::zeros(3,3,CV_32F);
    cameraM.at<double>(0,0) = 1012.3213482211537;
    cameraM.at<double>(0,2) = 970.4085456062247;
    cameraM.at<double>(1,1) = 1019.9488892836728;
    cameraM.at<double>(1,2) = 523.8022374391905;
    cameraM.at<double>(2,2) = 1.0;
    distC = Mat::zeros(1,5,CV_32F);
    R = Mat::zeros(3,3,CV_32F);
    T = Mat::zeros(3,1,CV_32F);

    if(calibrated){
        ifstream fp;
        fp.open("/home/ir/perception_ws/src/vicon_with_camera/data/calibrated.txt");
        double temp;
        for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            fp>>temp;
            R.at<double>(i,j) = temp;
        }
        }
        for(int i=0;i<3;i++){
            fp>>temp;
            T.at<double>(i) = temp;
        }
    }
    else{
        num_mouse = 0;
        max_mouse = 4;
    }
}

vicon::~vicon(){

}

void vicon::callback(const ImageConstPtr& image, const viconMsgType::ConstPtr& viconMsg)
{
  // Solve all of perception here...
	ROS_INFO_STREAM("COOL");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
#if 0
        imshow("test",cv_ptr->image);
        waitKey(1);
#endif
        if(calibrated){
            //show the position of the markers
            vector<Point2f> markerImg;
            getPositions(markerImg,*viconMsg);
            for(size_t i=0;i<markerImg.size();i++)
                circle(cv_ptr->image,markerImg[i],2,Scalar(0,0,255));
        }
        else{
            //Do calibration
            int kPad = getche();
            if(kPad == 32){
                char img_f[100] = "";
                char vicon_f[100] = "";
                sprintf(img_f,img_path.c_str(),num);
                sprintf(vicon_f,vicon_path.c_str(),num);
                imwrite(img_f,cv_ptr->image);
                ofstream vicon_fi;
                vicon_fi.open(vicon_f);
                vicon_fi<<viconMsg->transform.translation.x<<" "<<viconMsg->transform.translation.y<<" "<<viconMsg->transform.translation.z<<" "<<viconMsg->transform.rotation.x<<" "<<viconMsg->transform.rotation.y<<" "<<viconMsg->transform.rotation.z<<" "<<viconMsg->transform.rotation.w;
                vicon_fi.close();
                bool mouseAc = false;
                while(!mouseAc){
                    namedWindow("Mouse Window", 1);
                    setMouseCallback("Mouse Window", CallBackFunc, NULL);
                    imshow("Mouse Window", cv_ptr->image);
                    waitKey(0);
                    mouseAc = true;
                    num_mouse++;
                    num++;
                }
                if(num_mouse==max_mouse){

                }
            }
#if 0
            imshow("performing calibration",cv_ptr->image);
            waitKey(1);
#endif
        }
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void vicon::getPositions(vector<Point2f> &markerImg, viconMsgType viconMsg){
    vector<Point3f> objectPoints;
    //TODO from msg get the 3d points
    //Project the 3d points on the image plane
    Mat tvec = T.clone();
    Mat rvec;
    Rodrigues(R,rvec);
    projectPoints(objectPoints,rvec,tvec,cameraM,distC,markerImg); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vicon_with_cam_node");
	vicon VC;
	ros::spin();

	return 0;
}

