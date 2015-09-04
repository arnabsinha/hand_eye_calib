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
    int nM = 4;
    vector<Point3f> calibWorldPoints;// = Mat::zeros(nM*numIm,3,CV_32F);
    vector<Point3f> calibImagePoints;// = Mat::zeros(nM*numIm,3,CV_32F);
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
            Point3f a, b;
            a.x = tempx;
            a.y = tempy;
            a.z = tempz;
            calibWorldPoints.push_back(a);

            iF>>tempxi>>tempyi;
            b.x = tempxi;
            b.y = tempyi;
            b.z = 1.0;
            calibImagePoints.push_back(b);
        }
        vF.close();
        iF.close();
    }
//    cout<<calibWorldPoints<<endl;
//    cout<<calibImagePoints<<endl;
    Mat cameraM = Mat::zeros(3,3,CV_32F), distC = Mat::zeros(1,5,CV_32F);
    cameraM.at<float>(0,0) = 1012.3213482211537;
    cameraM.at<float>(0,2) = 970.4085456062247;
    cameraM.at<float>(1,1) = 1019.9488892836728;
    cameraM.at<float>(1,2) = 523.8022374391905;
    cameraM.at<float>(2,2) = 1.0;

    Mat rvec, tvec;
//    solvePnPRansac(calibWorldPoints,calibImagePoints,cameraM,distC,rvec,tvec);
    Mat M = Mat::zeros(3,4,CV_32F);
    Mat inl = Mat::zeros(calibImagePoints.size(),1,CV_8U);
    estimateAffine3D(calibWorldPoints,calibImagePoints, M,inl,3,0.9);
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
