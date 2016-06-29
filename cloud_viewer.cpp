#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
//PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>

//#include <pcl/surface/mls.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/registration/transforms.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//Labeling
#include "Labeling.h"

//#define normal
#define color
#define cut
#define plane
//#define file
#define image
//#define labeling

int user_data;
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Read PCD data
    pcl::io::loadPCDFile ("0001_cloud.pcd", *cloud);

    //downsampling    
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.001, 0.001, 0.001);
    sor.filter (*cloud2);

#ifdef cut 
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud2);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.25, 1.0);
    pass.setFilterLimitsNegative(true);
    pass.filter (*cloud3);
#endif

#ifdef plane
    // Plane model segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);

    seg.setInputCloud (cloud3);
    seg.segment (*inliers, *coefficients);    
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud3);
    extract.setIndices(inliers);
    extract.setNegative(true); //Trueの場合出力は検出された壁以外のデータ falseの場合は壁のデータ
    extract.filter(*cloud3);
#endif

#ifdef color    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloth (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cloud3->points.size (); ++i)
    {
        //This is cloth
        if( cloud3->points[i].r > 0 && cloud3->points[i].r < 50 && cloud3->points[i].g > 0 && cloud3->points[i].g < 50 && cloud3->points[i].b < 65 && cloud3->points[i].b > 0)
        {
            pcl::PointXYZRGB temp;
            temp = cloud3->points[i];
            //temp.y = cloud.points[i].y;
            //temp.z = cloud.points[i].z;
            //temp.r = cloud.points[i].r;
            //temp.g = cloud.points[i].g;
            //temp.b = cloud.points[i].b;
            cloth->push_back(temp);
            //pcl::pointCloud.point.erase(index)???
            //pcl::PointCloud<PointXYZRGB>::erase(index);
            //cloud.points[i].r = 0;
            //cloud.points[i].g = 0;
            //cloud.points[i].b = 200;    
        }
    }
    cloud3 = cloth;
#endif

    // cloth length
    double max_x = 0;
    double max_y = 0;
    double max_z = 0;
    double min_x = 0;
    double min_y = 0;
    double min_z = 0;

    for (size_t i = 0; i < cloud3->points.size (); ++i)
    {
        if (max_z > cloud3->points[i].z)
            max_z = max_z;
        else
            max_z = cloud3->points[i].z;

        if (max_y > cloud3->points[i].y)
            max_y = max_y;
        else
            max_y = cloud3->points[i].y;

        if (max_x > cloud3->points[i].x)
            max_x = max_x;
        else
            max_x = cloud3->points[i].x;

        if (min_z < cloud3->points[i].z)
            min_z = min_z;
        else
            min_z = cloud3->points[i].z;

        if (min_y < cloud3->points[i].y)
            min_y = min_y;
        else
            min_y = cloud3->points[i].y;

        if (min_x < cloud3->points[i].x)
            min_x = min_x;
        else
            min_x = cloud3->points[i].x;
    }

    std::cout << "max_x: " << max_x << std::endl;
    std::cout << "max_y: " << max_y << std::endl;
    std::cout << "max_z: " << max_z << std::endl;
    std::cout << "min_x: " << min_x << std::endl;
    std::cout << "min_y: " << min_y << std::endl;
    std::cout << "min_z: " << min_z << std::endl;

    std::cout << "delta_x: " << max_x - min_x << std::endl;
    std::cout << "delta_y: " << max_y - min_x << std::endl;
    std::cout << "delta_z: " << max_z - min_z << std::endl;
    

        double tmp_d = coefficients->values[3];

        double trans_x = std::abs(min_x)+0.5;
        double trans_y = std::abs(min_y)+0.1;
        double trans_z = 1.0 - std::abs(tmp_d);

        Eigen::Matrix4f mat_trans;
        mat_trans << \
            1.0, 0.0, 0.0, trans_x, \
            0.0, 1.0, 0.0, trans_y, \
            0.0, 0.0, 1.0, trans_z, \
            0.0, 0.0, 0.0, 1.0;

        //---transform
        pcl::transformPointCloud(*cloud3, *cloud3, mat_trans);


#ifdef image
    //------------------------------------------Create range image from cloud
    //---set trimming value (distance from cloud origin) unit:[m]
    double tr_width = 1.5;
    double tr_height = 1.5;

        //---set number of pixel per meter
    int num_pix = 400;
    int height = num_pix*tr_height;
    int width = num_pix*tr_width;

    //---create matrix
    cv::Mat range_image(height, width, CV_8U, 255);

    //---input cloud depth data to matrix
    for (size_t j = 0; j < cloud3->points.size (); ++j)
    {
        if(cloud3->points[j].x >= 0 && cloud3->points[j].x < tr_width)
        {
            if(cloud3->points[j].y >= 0 && cloud3->points[j].y < tr_height)
            {
                double val_x = num_pix*cloud3->points[j].x;
                double val_y = num_pix*cloud3->points[j].y;
                double val_z = (255/std::abs(tmp_d))*cloud3->points[j].z;
                // float val_z = cloud.points[j].z;
                int pix_x = val_x;
                int pix_y = val_y;
                int intensity = val_z;
                if(intensity >= 0 && intensity <= 255)
                {
                    uchar *p = range_image.ptr<uchar>(pix_y);
                    if(intensity < p[pix_x])
                    // if(val_z < p[pix_x])
                    { 
                        p[pix_x] = intensity;
                        // p[pix_x] = val_z;
                    }
                }
            }
        }
    }
/*
    cv::namedWindow("range",CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::imshow("range", range_image);
    cv::waitKey(0);
*/

    //------------------------------------------inpainting
    cv::Mat mask = cv::Mat::zeros(range_image.size(), CV_8U);
    cv::threshold(range_image, mask, 250, 255, CV_THRESH_TOZERO);
    for(int k = 0; k < height; ++k)
        {
            uchar *q = mask.ptr<uchar>(k);
            for(int n = 0; n < width; ++n)
                {
                    if(q[n] == 255)
                        {
                            q[n] = 0;
                        }
                    else
                        {
                            break;
                        }
                }
        }
    for(int k = 0; k < height; ++k)
        {
            uchar *q = mask.ptr<uchar>(k);
            for(int n = width-1; n >=0 ; --n)
                {
                    if(q[n] == 255)
                        {
                            q[n] = 0;
                        }
                    else
                        {
                            break;
                        }
                }
        }

    cv::Mat range_inpainted = cv::Mat::zeros(range_image.size(), CV_8U);
    // do inpaint 
    cv::inpaint(range_image, mask, range_inpainted, 5.0, CV_INPAINT_NS);
#endif

#ifdef labeling
    cv::Mat gray;

    cv::threshold(range_inpainted, gray, 250, 255, cv::THRESH_BINARY_INV|cv::THRESH_OTSU);

    cv::Mat dst = cv::Mat(gray.rows, gray.cols, CV_16SC1);
    cv::Mat label = cv::Mat(gray.rows, gray.cols, CV_8UC1);

    LabelingBS lb;
    lb.Exec(gray.data, (short*)dst.data, gray.cols, gray.rows, true, 100);

    short val;
    for (int y = 0; y < label.rows; y++){
        for (int x = 0; x < label.cols; x++){
            val = dst.data[dst.step*y + 2 * x];
            if (val == 1) label.data[label.step*y + x] = 255;
            else label.data[label.step*y + x] = 0;
        }
    }

    RegionInfoBS *ri = lb.GetResultRegionInfo(0);

    int area = ri->GetNumOfPixels();

    std::cout << "area: " << area << std::endl;

#endif

#ifdef file
    std::ofstream ofs("cloth_data.csv",ios::app);
    ofs << "lg" << "," << max_x - min_x << "," << max_y - min_y << "," 
    << std::abs(tmp_d) << "," << area <<std::endl;
#endif

    
#ifdef normal
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud3);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 5cm
    ne.setRadiusSearch (0.03);
    
    //testing
    //pcl::PointCloud<pcl::PricipalCurvatures>::Ptr pcs (new pcl::PointCloud<pcl::PricipalCurvatures>);

    // Compute the features
    ne.compute (*cloud_normals);
#endif

    /*
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    //Set parameters
    mls.setInputCloud(cloud2);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);

    //Reconstruct
    mls.process(mls_points);
    */
    
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.5, 0.5, 0.5);
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud3);
    viewer.addPointCloud(cloud3,rgb,"cloud");
    
    //viewer.addPointCloud(cloud2,"cloud");

    /*
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud2, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud2, single_color, "normals");
    */

#ifdef normal
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud3, cloud_normals,100,0.02,"normals",0);
#endif 

    //viewer.addPointCloudPrincipalCurvatures<pcl::PointXYZ,pcl::Normal>(cloud2, cloud_normals, pcs, 100,0.02,"normals",0);

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }

#ifdef image
    cv::namedWindow("range",CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::imshow("range", range_image);
    cv::waitKey(0);
#endif

    return 0;
}