#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;


int main(int argc, char* argv[]){
    int numIm = atoi(argv[1]);
    string path = "/home/ir/perception_ws/src/vicon_with_camera/data/";
    string imgP = path + "points%d.txt";
    string viconP = path + "vicon%d.txt";
    string calibP = path + "calibrated.txt";
    int nM = atoi(argv[2]);
    Mat calibWorldPoints = Mat::zeros(nM*numIm,3,CV_32F);
    Mat calibImagePoints = Mat::zeros(nM*numIm,2,CV_32F);
    for(int i=0;i<numIm;i++){
        char imgF[100] = "";
        char viconF[100] = "";
        sprintf(imgF,imgP.c_str(),i);
        sprintf(viconF,viconP.c_str(),i);
//        cout<<imgF<<endl<<viconF<<endl;
        ifstream vF, iF;
        vF.open(viconF);
        iF.open(imgF);
//        cout<<viconF<<" "<<imgF<<endl;
        for(int j=0;j<nM;j++){
            float tempx, tempy, tempz, tempxi, tempyi;
            vF>>tempx>>tempy>>tempz;
//            cout<<i<<" "<<j<<" "<<i*numIm+j<<" "<<calibWorldPoints.rows<<endl;
            calibWorldPoints.at<float>(i*nM+j,0) = tempx/1000.0;
            calibWorldPoints.at<float>(i*nM+j,1) = tempy/1000.0;
            calibWorldPoints.at<float>(i*nM+j,2) = tempz/1000.0;
            iF>>tempxi>>tempyi;
            calibImagePoints.at<float>(i*nM+j,0) = tempxi;
            calibImagePoints.at<float>(i*nM+j,1) = tempyi;
        }
        vF.close();
        iF.close();
    }
//    cout<<calibWorldPoints<<endl;
//    cout<<calibImagePoints<<endl;
    Mat cameraM = Mat::zeros(3,3,CV_32F), distC = Mat::zeros(1,5,CV_32F);
#if 0
    cameraM.at<float>(0,0) = 1012.3213482211537;
    cameraM.at<float>(0,2) = 970.4085456062247;
    cameraM.at<float>(1,1) = 1019.9488892836728;
    cameraM.at<float>(1,2) = 523.8022374391905;
    cameraM.at<float>(2,2) = 1.0;
#endif
    string cameraFile = "/home/ir/kinect2WS/src/iai_kinect2/kinect2_bridge/data/507554242542/calib_color.yaml";
    FileStorage fs2(cameraFile.c_str(), FileStorage::READ);
    fs2["cameraMatrix"]>>cameraM;
    fs2["distortionCoefficients"]>>distC;
    fs2.release();

    Mat rvec, tvec;
    solvePnPRansac(calibWorldPoints,calibImagePoints,cameraM,distC,rvec,tvec);
//    solvePnP(calibWorldPoints,calibImagePoints,cameraM,distC,rvec,tvec);
    ofstream fp;
    fp.open(calibP.c_str());
    for(int i=0;i<3;i++)
        fp<<rvec.at<double>(i)<<" ";
    for(int i=0;i<3;i++)
        fp<<tvec.at<double>(i)<<" ";
    fp.close();
    return 0;
}
