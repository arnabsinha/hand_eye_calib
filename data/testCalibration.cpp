#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

vector<Point2f> imgPoints;
Mat viconImg;
void testImage(string imgP, string viconP, string calibP, int numIm, int nM, string imageW);

int main(int argc, char* argv[]){
    int numIm = atoi(argv[1]);
    int nM = atoi(argv[2]);
    string path = "/home/ir/perception_ws/src/vicon_with_camera/data/";
    string imgP = path + "image%d.png";
    string viconP = path + "vicon%d.txt";
    string calibP = path + "calibrated.txt";
    string imageW = path + "testResult%d.png";
    for(int i=0;i<numIm;i++)
        testImage(imgP,viconP,calibP,i, nM, imageW);
    return 0;
}


void testImage(string imgP, string viconP, string calibP, int numIm, int nM, string imageW){
    char img_f[100] = "";
    char vicon_f[100] = "";
    char calib_f[100] = "";
    char imageWr[100] = "";
    sprintf(img_f,imgP.c_str(),numIm);
    sprintf(vicon_f,viconP.c_str(),numIm);
    sprintf(imageWr,imageW.c_str(),numIm);
    Mat viconPoints = Mat::zeros(4,2,CV_32F);
    Mat I = imread(img_f);
    viconImg = I.clone();
    ifstream fpI, fpC;
    fpI.open(vicon_f);
    fpC.open(calibP.c_str());
    Mat rvec = Mat::zeros(3,1,CV_32F), tvec = rvec.clone();
//    Mat R = Mat::zeros(3,3,CV_32F);
    float temp;
    for(int i=0;i<3;i++){
        fpC>>temp;
        rvec.at<float>(i) = temp;
    }
//    R = R.t();
    for(int i=0;i<3;i++){
        fpC>>temp;
        tvec.at<float>(i) = temp;
    }
//    tvec = -R * tvec;
    cout<<rvec<<endl<<tvec<<endl;
//    Rodrigues(R,rvec);
    Mat cameraM = Mat::zeros(3,3,CV_32F), distC = Mat::zeros(1,5,CV_32F);
    string cameraFile = "/home/ir/kinect2WS/src/iai_kinect2/kinect2_bridge/data/507554242542/calib_color.yaml";
    FileStorage fs2(cameraFile.c_str(), FileStorage::READ);
#if 0
    cameraM = Mat::zeros(3,3,CV_32F);
    cameraM.at<float>(0,0) = 1012.3213482211537;
    cameraM.at<float>(0,2) = 970.4085456062247;
    cameraM.at<float>(1,1) = 1019.9488892836728;
    cameraM.at<float>(1,2) = 523.8022374391905;
    cameraM.at<float>(2,2) = 1.0;
    distC = Mat::zeros(1,5,CV_32F);
#endif
    fs2["cameraMatrix"]>>cameraM;
    fs2["distortionCoefficients"]>>distC;
    fs2.release();


/*** Prepare vicon Image ***/
//    vector<Point3f> worldPt;
//    vector<Point2f> imagePt;
    Mat worldPoints = Mat::zeros(nM,3,CV_32F);
    for(int i=0;i<nM;i++){
        float tempx, tempy, tempz;
        fpI>>tempx>>tempy>>tempz;
/*
        Point3f tempPt;
        tempPt.x = tempx;
        tempPt.y = tempy;
        tempPt.z = tempz;
        worldPt.push_back(tempPt);
*/
        worldPoints.at<float>(i,0) = tempx/1000.0;
        worldPoints.at<float>(i,1) = tempy/1000.0;
        worldPoints.at<float>(i,2) = tempz/1000.0;
    }
//    projectPoints(worldPt,rvec,tvec,cameraM,distC,imagePt);
    projectPoints(worldPoints,rvec,tvec,cameraM,distC,viconPoints);
    for (size_t i=0;i<nM;i++){
//        viconPoints.at<float>(i,0) = imagePt[i].x;
//        viconPoints.at<float>(i,1) = imagePt[i].y;
        int x = viconPoints.at<float>(i,0);
        int y = viconPoints.at<float>(i,1);
        cout<<i<<" "<<x<<" "<<y<<endl;
        circle(viconImg,Point(x,y),(i+1)*2,Scalar(i*50,0,255),(i+1)*2,8);
    }
    imshow("test",viconImg);
    waitKey(0);
    imwrite(imageWr,viconImg);
/*** ********** ***/

    fpI.close();

}
