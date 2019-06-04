// 该文件将打开给定的视频文件，并将图像传递给ORB-SLAM2进行定位

// 需要opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco.hpp>
#include <fstream>
#include "opencv2/calib3d.hpp" //s
// ORB-SLAM的系统接口
#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>
#include "Converter.h"
using namespace std;

// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./myvideo_p.yaml";
string vocFile = "../../../Vocabulary/ORBvoc.bin";

// 视频文件
string videoFile = "./navinfo_park_maker_ipm.avi";

std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    //std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

std::clock_t getTimeStamp_s()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::clock_t timestamp = tmp.count();
    //std::clock_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

static bool readDetectorParameters(string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

/**
 */
static bool readCameraParameters(string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


int main(int argc, char **argv) {
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);

    // 获取视频图像
    cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.

    // 记录系统时间
    auto start = chrono::system_clock::now();
int count=0;
    ofstream f_aruco;
    f_aruco.open("SaveARUcoFrameTrajectoryTUM.txt");
    f_aruco<< fixed;
    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if ( frame.data == nullptr )
            break;
        count++;
        // rescale because image is too large

        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(640,360));

        //        auto now = chrono::system_clock::now();
        //        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        //        SLAM.TrackMonocular(frame_resized, double(timestamp.count())/1000.0);


        time_t myTimeStamp=getTimeStamp();//        clock_t myTimeStamp=getTimeStamp_s();
//        cout<<setprecision(6)<<myTimeStamp/1000.0<<endl;//1556450562.840000  setprecision(6)
        double myTimeStampUsed=myTimeStamp/1000.0;

        vector< int > ids;
        float markerLength=0.44;
        vector< vector< cv::Point2f > > corners, rejected;
        vector< cv::Vec3d > rvecs, tvecs;
        cv::Ptr<cv::aruco::Dictionary> dictionary =
                cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(0));//dictionaryId
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
            bool readdetetOk = readDetectorParameters("../detector_params.yml", detectorParams);
        if(!readdetetOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }

        cv::Mat camMatrix, distCoeffs;
            bool readcamOk = readCameraParameters("../camera_p.yml", camMatrix, distCoeffs);
            if(!readcamOk) {
                cerr << "Invalid camera file" << endl;
                return 0;
            }
        // detect markers and estimate pose
        cv::aruco::detectMarkers(frame_resized, dictionary, corners, ids, detectorParams, rejected);
        if(ids.size() > 0)
        {
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                                 tvecs);
            cv::aruco::drawDetectedMarkers(frame_resized, corners, ids);
            for (int i = 0; i < ids.size(); i++)
            {
                if(10==ids[i])
                { cv::Mat rvec(3, 1, CV_32FC1);//convert rotation3x3 ? unused
                    rvec.at<float>(0) = rvecs[i][0];
                    rvec.at<float>(1) = rvecs[i][1];
                    rvec.at<float>(2) = rvecs[i][2];
                    cv::Mat rotation3x3 = cv::Mat::zeros(3, 3, CV_32FC1);
                    cv::Rodrigues(rvec,rotation3x3);
                    std::vector<float>qr(4);
                    qr= ORB_SLAM2::Converter::toQuaternion(rotation3x3);

                    f_aruco << setprecision(6) << myTimeStampUsed<< setprecision(7) << " " << tvecs[i][0]<< " " << tvecs[i][1] << " " << tvecs[i][2]
                            << " " << qr[0] << " " << qr[1] << " " << qr[2] << " " << qr[3] << endl;
                }
            }
        }


        cout << endl << "trajectory saved!" << endl;
        SLAM.TrackMonocular(frame_resized,myTimeStampUsed);//myTimeStampUsed,double(myTimeStamp)/1000.0
        // Save camera trajectory  每个有效关键帧存储
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory2.txt");//SLAM.SaveKeyFrameTrajectoryTUM("./data.txt");
        SLAM.SaveKeyFrameTrajectoryTUMCount("KeyFrameTrajectoryCount.txt",count);
        cv::waitKey(30);
    }
    f_aruco.close();
    //    // Save map   not add/not work  XXXXXXXX
//        SLAM.SaveMap("Slam_latest_Map.bin");
    // Save camera trajectory  每个有效关键帧存储,整个结束存储
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    SLAM.Shutdown();
    return 0;
}

