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


using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

class vicon{
	private:
		ros::NodeHandle nh;
		message_filters::Subscriber<Image> *image_sub;
		message_filters::Subscriber<geometry_msgs::TransformStamped> *info_sub;
		typedef sync_policies::ApproximateTime<Image, geometry_msgs::TransformStamped> MySyncPolicy;
		Synchronizer<MySyncPolicy> *sync;
	public:
		void callback(const ImageConstPtr& image, const geometry_msgs::TransformStampedConstPtr& cam_info);
		vicon();
		~vicon();
};

vicon::vicon()
	:nh("~")
{
	image_sub = new message_filters::Subscriber<Image> (nh,"/kinect2/rgb_rect/image",1);
	info_sub = new message_filters::Subscriber<geometry_msgs::TransformStamped> (nh,"/vicon/astarTest/astarTest",1);
	sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(100), *image_sub, *info_sub);
	sync->registerCallback(boost::bind(&vicon::callback, this, _1, _2));
}

vicon::~vicon(){

}

void vicon::callback(const ImageConstPtr& image, const geometry_msgs::TransformStampedConstPtr& cam_info)
{
  // Solve all of perception here...
	ROS_INFO_STREAM("COOL");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        imshow("test",cv_ptr->image);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vicon_with_cam_node");
	vicon VC;
	ros::spin();

	return 0;
}

