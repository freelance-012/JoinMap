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

int main() {
    vector<cv::Mat> colorImgs, dispImgs;    // 彩色图和视差图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿

    ifstream fin("../pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }


    CsvFileReader csvFileReader;
    for ( int i=0; i<4; i++ )
    {
        for(int j=144; j<145; ++j) {
            boost::format fmt( "../%s/%d/%06d.%s" ); //图像文件格式

            cv::Mat originMat = cv::imread( (fmt%"grey"%(i+1)%(j)%"png").str() );


            //dispImgs.push_back( cv::imread( (fmt%"disp_csv"%(i+1)%"png.csv").str(), -1 )); // 使用-1读取原始图像
            vector<vector<double>> dispVec;

            cout << (fmt%"disp_csv"%(i+1)%(j)%"png.csv").str() << endl;
            csvFileReader.csvFileReader((fmt%"disp_csv"%(i+1)%(j)%"png.csv").str(), dispVec);

            // vector<vector<double>> 转 Mat<double>()
            cv::Mat dispMat = cv::Mat(dispVec.size(), dispVec[0].size(), CV_64F);
            for(int row=0; row<dispVec.size(); ++row) {
                for(int col=0; col<dispVec[row].size(); ++col) {
                    dispMat.ptr<double>(row)[col] = dispVec[row][col];
                }
            }

            cout << "1" << endl;


            if(i==1) {
                cv::transpose(dispMat, dispMat);
                cv::flip(dispMat, dispMat, 0);

                cv::transpose(originMat, originMat);
                cv::flip(originMat, originMat, 0);

            }
            else {
                cv::transpose(dispMat, dispMat);
                cv::flip(dispMat, dispMat, 1);

                cv::transpose(originMat, originMat);
                cv::flip(originMat, originMat, 1);
            }

            colorImgs.push_back(originMat);
            dispImgs.push_back(dispMat);


            double data[6] = {0};
            for ( auto& d:data )
                fin>>d;

            cout << "1" << endl;
            Eigen::Vector3d rotatedVec(data[3], data[4], data[5]);
            double theta = rotatedVec.norm();
            Eigen::Vector3d e (data[3]/theta, data[4]/theta, data[5]/theta);

            Eigen::AngleAxisd t_V(theta, e);


            Eigen::Quaterniond q;
            q = t_V;

            Eigen::Isometry3d T(q);
            T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
            poses.push_back( T );
        }
    }

    cout << colorImgs.size() << endl;
    cout << dispImgs.size() << endl;


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
    for ( int i=0; i<1; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl;
        cv::Mat color = colorImgs[i];
        cv::Mat disp = dispImgs[i];

        cout << color.cols << " " << color.rows << endl;
        cout << disp.cols << " " << disp.rows << endl;

        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = disp.ptr<unsigned short> ( v )[u]; /// 视差值
                if ( d==0 ) d=1; // 为0表示没有测量到

                Eigen::Vector3d point;
                //point[2] = double(d)/depthScale;
                //point[2] = 1;
                point[2] = -fx[i] * baseline[i] / (depthScale * d);
                point[0] = (u-cy[i])*point[2]/fy[i];
                point[1] = (v-cx[i])*point[2]/fx[i];

                point[0] = -point[0];
                point[1] = -point[1];

                Eigen::Vector3d pointWorld;
                if(i == 0) {
                    pointWorld = point;
                }
                else {
                    pointWorld = T*point;
                }

                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels() ];
                p.r = color.data[ v*color.step+u*color.channels() ];
                pointCloud->points.push_back( p );
            }
    }

    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;


    pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

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
