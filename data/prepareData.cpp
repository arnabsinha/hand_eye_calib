#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void prepareData(string imgP, string viconP, string calibP, int numIm, int nM);

vector<Point2f> imgPoints;
vector<Point2f> ear_imgP;
Mat viconImg;

int main(int argc, char* argv[]){
    int numIm = atoi(argv[1]);
    int nM = atoi(argv[2]);
    string path = "/home/ir/perception_ws/src/vicon_with_camera/data/";
    string imgP = path + "image%d.png";
    string viconP = path + "vicon%d.txt";
    string calibP = path + "points%d.txt";
    for(int i=0;i<numIm;i++){
        prepareData(imgP,viconP,calibP,i,nM);
    }
    return 0;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata){
    if  ( event == EVENT_LBUTTONDOWN )
    {
        Point2f pt;
        pt.x = x;
        pt.y = y;
        imgPoints.push_back(pt);
        circle(viconImg,pt,(int)(imgPoints.size())+1,Scalar(0,0,255),(int)(imgPoints.size())+1);
        imshow("Mouse Window",viconImg);
    }
    if  ( event == EVENT_RBUTTONDOWN )
    {
        for(size_t i=0;i<ear_imgP.size();i++){
            Point2f pt = ear_imgP[i];
            imgPoints.push_back(pt);
            circle(viconImg, pt, (i+1)*2, Scalar(0,0,255), i+1);
        }
        imshow("Mouse Window",viconImg);
    }
}

void prepareData(string imgP, string viconP, string calibP, int numIm, int nM){
        imgPoints.clear();
        char img_f[100] = "";
        char vicon_f[100] = "";
        char calib_f[100] = "";
        sprintf(img_f,imgP.c_str(),numIm);
        sprintf(vicon_f,viconP.c_str(),numIm);
        sprintf(calib_f,calibP.c_str(),numIm);
        Mat viconPoints = Mat::zeros(nM,2,CV_64F);
        Mat I = imread(img_f);
        viconImg = I.clone();
        ifstream fpI;
        fpI.open(vicon_f);

/*** Prepare vicon Image ***/
        for(int i=0;i<nM;i++){
            double tempx, tempy, tempz;
            fpI>>tempx>>tempy>>tempz;
            viconPoints.at<double>(i,0) = tempx/1000.0;
            viconPoints.at<double>(i,1) = tempy/1000.0;
        }
        PCA pca_analysis(viconPoints, Mat(), CV_PCA_DATA_AS_ROW);
        Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));
        vector<double> dist;
        double max_dist=-1;
        for(size_t i=0;i<nM;i++){
            dist.push_back(fabs(viconPoints.at<double>(i,0)-pos.x)+fabs(viconPoints.at<double>(i,1)-pos.y));
            if(max_dist<dist[dist.size()-1])
                max_dist = dist[dist.size()-1];
        }
        int nr = viconImg.rows, nc = viconImg.cols;
        double scale = max_dist/(double)(nr/3);
        int xT = nr/3;
        int yT = nc/3;
        for (size_t i=0;i<nM;i++){
            int x = (viconPoints.at<double>(i,0)-pos.x)/scale + xT;
            int y = (viconPoints.at<double>(i,1)-pos.y)/scale + yT;
//        cout<<i<<" "<<x<<" "<<y<<endl;
            circle(viconImg,Point(x,y),(i+1)*2,Scalar(i*50,0,255),(i+1)*2,8);
        }
/*** ********** ***/

        ofstream fpO;
        fpO.open(calib_f);
        namedWindow("Mouse Window", 1);
        setMouseCallback("Mouse Window", CallBackFunc);
        imshow("Mouse Window", viconImg);
        waitKey(0);
        ear_imgP.clear();
        for(int i=0;i<nM;i++){
            fpO<<imgPoints[i].x<<" "<<imgPoints[i].y<<endl;
            Point2f pt = imgPoints[i];
            ear_imgP.push_back(pt);
        }
        fpI.close();
        fpO.close();

}
