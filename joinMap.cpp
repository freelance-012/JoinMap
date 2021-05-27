#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "CsvFileReader.h"

int main( int argc, char** argv ) {
    vector<cv::Mat> colorImgs, dispImgs;    // 彩色图和视差图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿

    boost::format fmt( "../pose/%d&%d/%s.txt" ); //图像文件格式
    string poseR = (fmt%4%5%"R").str();
    ifstream finR(poseR);
    if (!finR)
    {
        cerr<< poseR << " cannot find！"<< endl;
        return 1;
    }

    string poseT = (fmt%4%5%"t").str();
    ifstream finT(poseT);
    if (!finT)
    {
        cerr<< poseT << " cannot find！"<< endl;
        return 1;
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d t; // 平移向量

    CsvFileReader csvFileReader;
    for ( int i=0; i<5; i++ )
    {
//        if(i!=0) {
//            continue;
//        }

        for(int j=40; j<41; ++j) {
            boost::format fmt( "../%s/%d/%06d.%s" ); //图像文件格式

            cv::Mat originMat = cv::imread( (fmt%"grey"%(i+1)%(j)%"png").str() );


            //dispImgs.push_back( cv::imread( (fmt%"disp_csv"%(i+1)%"png.csv").str(), -1 )); // 使用-1读取原始图像
//            vector<vector<double>> dispVec;
//
//            cout << (fmt%"disp_csv"%(i+1)%(j)%"png.csv").str() << endl;
//            csvFileReader.csvFileReader((fmt%"disp_csv"%(i+1)%(j)%"png.csv").str(), dispVec);
//
//            // vector<vector<double>> 转 Mat<double>()
//            cv::Mat dispMat = cv::Mat(dispVec.size(), dispVec[0].size(), CV_64F);
//            for(int row=0; row<dispVec.size(); ++row) {
//                for(int col=0; col<dispVec[row].size(); ++col) {
//                    dispMat.ptr<double>(row)[col] = dispVec[row][col];
//                }
//            }
//
//            cout << "1" << endl;

//
//            if(i==1) {
//                cv::transpose(dispMat, dispMat);
//                cv::flip(dispMat, dispMat, 0);
//
//                cv::transpose(originMat, originMat);
//                cv::flip(originMat, originMat, 0);
//
//            }
//            else {
//                cv::transpose(dispMat, dispMat);
//                cv::flip(dispMat, dispMat, 1);
//
//                cv::transpose(originMat, originMat);
//                cv::flip(originMat, originMat, 1);
//            }

            colorImgs.push_back(originMat);
//            dispImgs.push_back(dispMat);



            double data[6] = {0};

            finR >> R(0,0) >> R(0,1) >> R(0,2)
                    >> R(1,0) >> R(1,1) >> R(1,2)
                    >> R(2,0) >> R(2,1) >> R(2,2);

            cout << R << endl;


            finT >> data[0] >> data[1] >> data[2];

            // 先求旋转向量 再转为四元数
//            Eigen::Vector3d rotatedVec(data[3], data[4], data[5]);
//            double theta = rotatedVec.norm();
//            Eigen::Vector3d e (data[3]/theta, data[4]/theta, data[5]/theta);
//
//            Eigen::AngleAxisd t_V(theta, e);
//
//            R = R.inverse();
            Eigen::Quaterniond q(R);
            Eigen::Isometry3d T(q);

            Eigen::Vector3d t = Eigen::Vector3d( data[0], data[1], data[2] ); // 平移向量
//            t = -R*t;
            T.pretranslate( t);


            if(i==3) {

//                Eigen::Vector3d rotatedVec(0, 0, 0);
                double theta = 0;//rotatedVec.norm();
                Eigen::Vector3d e (0,0,0);

                Eigen::AngleAxisd t_V(theta, e);

                q = t_V;
                T = q;
                T.pretranslate(Eigen::Vector3d(0,0,0));

            }


            poses.push_back( T );
        }
    }

    cout << colorImgs.size() << endl;
//    cout << dispImgs.size() << endl;


    // 计算点云并拼接
    // 相机内参
    //  5 1 2 3 4 //  1 2 3 4 5  cameraMatrix[0]
    double cx[5] = {675.0717351159951, 561.1870204672966, 701.2909123824906, 666.0068330871056, 712.9304773426148};
    double cy[5] = {473.7360864674622, 469.245484628616, 478.5841860710722, 433.2917302356004, 434.0844654021636};
    double fx[5] = {696.2603803238521, 690.5599040699601, 709.0662610350147, 693.7897007083014, 691.0149744002296};
    double fy[5] = {697.1236739851626, 691.4353368472873, 710.2971326454953, 694.8215660049831, 691.6301252481686};
    double baseline[5] = {81.07294026711735, 81.4114004427543, 82.69353851443422, 81.47410529635283, 82.29917261819834};
    double depthScale = 1000.0;

    cout<<"正在将图像转换为点云..."<<endl;

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );
    for ( int i=0; i<5; i++ )
    {
        if(i!=3 && i!=4) {
            continue;
        }

        cout<<"转换图像中: "<<i+1<<endl;
        cv::Mat color = colorImgs[i];
//        cv::Mat disp = dispImgs[i];

        cout << color.cols << " " << color.rows << endl;
//        cout << disp.cols << " " << disp.rows << endl;

        Eigen::Isometry3d T = poses[i];

        for (int row = 0; row < color.rows; row++)
        {
//            if(row < 400) {
//                continue;
//            }
            for (int col=0; col < color.cols; col++)
            {

//                if(col < 100 ) continue;

                // 亮度太大 视差效果不好，滤掉
//                if (color.ptr<uchar>(row)[col*3] > 240 && col > 400 && row < 700) { // 灰度图就更加粗略跳过
//                    continue;
//                }

                double d = 2;
//                double d = disp.ptr<double>(row)[col];
//                d = (int)(d*100)/100.0;
                //float d = m_DispVec[row][col];

                // 认为 位置高的一般都远， 如果位置在前500行，并且视差大于阈值  滤掉
//                if(row > 500 && d >30) {
//                    continue;
//                }
//                if(d < 0.5) {
//                    continue;
//                }



                Eigen::Vector3d point;
                point[2] = 1;
                point[2] = fx[i] * baseline[i] / (depthScale * d);
                point[0] = (col-cy[i])*point[2]/fy[i];
                point[1] = (row-cx[i])*point[2]/fx[i];

                point[0] = -point[0];
                point[1] = -point[1];

                Eigen::Vector3d pointWorld;
                if(i == 3) {
                    pointWorld = point;
                }
                else {
                    pointWorld = T*point;
                }

                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ row*color.step+col*color.channels() ];
                p.g = color.data[ row*color.step+col*color.channels() ];
                p.r = color.data[ row*color.step+col*color.channels() ];
//                p.b = color.ptr<uchar>(row)[col*3];
//                p.g = color.ptr<uchar>(row)[col*3];
//                p.r = color.ptr<uchar>(row)[col*3];
                pointCloud->points.push_back( p );

            }
        }

    }

    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;


    pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (1);
    viewer->initCameraParameters ();
    viewer->removePointCloud("sample cloud");
    if(!viewer->updatePointCloud (pointCloud))
    {
        viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
    }
    while(!viewer->wasStopped()) {
        viewer->spinOnce (0);

    }

    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    return 0;
}
