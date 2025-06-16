/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>                 //  <-- MLS support
#include "common_lib.h"
#include "import_pcd.hpp"

class LidarDetect
{
private:
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double circle_radius_; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr custom_plane_cloud_;
    bool use_custom_plane_cloud_;
    std::string custom_plane_path_;

    // 存储中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_z0_cloud_;

public:
    ros::Publisher filtered_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher aligned_pub_;
    ros::Publisher edge_pub_;
    ros::Publisher center_z0_pub_;
    ros::Publisher center_pub_;

    LidarDetect(ros::NodeHandle &nh, Params &params)
        : filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          plane_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        /* added use_custom_plane_cloud and custom_plane_path parameters to specify the custom plane cloud and its path */
        use_custom_plane_cloud_ = params.use_custom_plane;
        custom_plane_path_ = params.custom_plane_path;

        filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);
        aligned_pub_ = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
        edge_pub_ = nh.advertise<sensor_msgs::PointCloud2>("edge_cloud", 1);
        center_z0_pub_ = nh.advertise<sensor_msgs::PointCloud2>("center_z0_cloud", 10);
        center_pub_ = nh.advertise<sensor_msgs::PointCloud2>("center_cloud", 10);
    }

    void detect_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud)
    {
        /* added cloud_params to iterate over a number of parameters sets in a single pass */
        std::vector<std::map<std::string, double>> cloud_params = {
             {{"leaf_size",0.002},{"radius_search",0.020},{"mls_radius",0.03},
             {"distance_threshold",0.003},{"min_cluster_size",80},{"max_cluster_size",600},
             {"cluster_tolerance",0.006},{"angle_threshold",M_PI/2}},
            // medium (≈5 mm spacing, Velodyne VLP-16 at 15 m)
            {{"leaf_size",0.005},{"radius_search",0.030},{"mls_radius",0.03},
             {"distance_threshold",0.006},{"min_cluster_size",50},{"max_cluster_size",1000},
             {"cluster_tolerance",0.012},{"angle_threshold",M_PI/2}},
            // coarse (≈10 mm spacing, mobile mapping)
            {{"leaf_size",0.010},{"radius_search",0.050},{"mls_radius",0.03},
             {"distance_threshold",0.015},{"min_cluster_size",30},{"max_cluster_size",3000},
             {"cluster_tolerance",0.04},{"angle_threshold",M_PI/2}}
         };

        int trial_number = 0;

        for(auto& param : cloud_params) {
            trial_number++;
        

            if (!use_custom_plane_cloud_) {
            // 1. X、Y、Z方向滤波
                filtered_cloud_->reserve(cloud->size());

                pcl::PassThrough<pcl::PointXYZ> pass_x;
                pass_x.setInputCloud(cloud);
                pass_x.setFilterFieldName("x");
                pass_x.setFilterLimits(x_min_, x_max_);  // 设置X轴范围
                pass_x.filter(*filtered_cloud_);
            
                pcl::PassThrough<pcl::PointXYZ> pass_y;
                pass_y.setInputCloud(filtered_cloud_);
                pass_y.setFilterFieldName("y");
                pass_y.setFilterLimits(y_min_, y_max_);  // 设置Y轴范围
                pass_y.filter(*filtered_cloud_);
            
                pcl::PassThrough<pcl::PointXYZ> pass_z;
                pass_z.setInputCloud(filtered_cloud_);
                pass_z.setFilterFieldName("z");
                pass_z.setFilterLimits(z_min_, z_max_);  // 设置Z轴范围
                pass_z.filter(*filtered_cloud_);
            
                ROS_INFO("Filtered cloud size: %ld", filtered_cloud_->size());

                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(filtered_cloud_);
                voxel_filter.setLeafSize(param["leaf_size"], param["leaf_size"], param["leaf_size"]);
                voxel_filter.filter(*filtered_cloud_);
                ROS_INFO("Filtered cloud size: %ld", filtered_cloud_->size()); 
                
            } else { /* if use_custom_plane_cloud is true, use the custom plane cloud */
                custom_plane_cloud_ = import_pcd(custom_plane_path_);
                filtered_cloud_ = custom_plane_cloud_;

                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(filtered_cloud_);
                voxel_filter.setLeafSize(param["leaf_size"], param["leaf_size"], param["leaf_size"]);
                voxel_filter.filter(*filtered_cloud_);
                ROS_INFO("Filtered cloud size: %ld", filtered_cloud_->size()); 
            } 

            if (filtered_cloud_->empty()) {
                ROS_WARN("[%d] Filtered cloud empty – check passthrough limits / leaf size", trial_number);
                continue;
            }

            // 2. 平面分割
            plane_cloud_->clear();

            pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
            plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
            plane_segmentation.setMethodType(pcl::SAC_RANSAC);
            plane_segmentation.setDistanceThreshold(0.01);  // 平面分割阈值
            plane_segmentation.setInputCloud(filtered_cloud_);
            plane_segmentation.segment(*plane_inliers, *plane_coefficients);
        
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(filtered_cloud_);
            extract.setIndices(plane_inliers);
            extract.filter(*plane_cloud_);
            ROS_INFO("Plane cloud size: %ld", plane_cloud_->size());
        
            // 3. 平面点云对齐   
            aligned_cloud_->clear();

            Eigen::Vector3d normal(plane_coefficients->values[0],
                plane_coefficients->values[1],
                plane_coefficients->values[2]);
            normal.normalize();
            Eigen::Vector3d z_axis(0, 0, 1);

            Eigen::Vector3d axis = normal.cross(z_axis);
            double angle = acos(normal.dot(z_axis));

            Eigen::AngleAxisd rotation(angle, axis);
            Eigen::Matrix3d R = rotation.toRotationMatrix();

            // 应用旋转矩阵，将平面对齐到 Z=0 平面
            float average_z = 0.0;
            int cnt = 0;
            for (const auto& pt : *plane_cloud_) {
                Eigen::Vector3d point(pt.x, pt.y, pt.z);
                Eigen::Vector3d aligned_point = R * point;
                aligned_cloud_->push_back(pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
                average_z += aligned_point.z();
                cnt++;
            }
            average_z /= cnt;

            ROS_INFO_STREAM("Aligned cloud size: " << aligned_cloud_->size());

            /* apply MLS to densify the aligned cloud; not sure if this even works */
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
            mls.setComputeNormals(false);
            mls.setInputCloud(aligned_cloud_);
            mls.setPolynomialOrder(2);
            mls.setSearchMethod(tree);
            mls.setSearchRadius(param.at("mls_radius")); // e.g., 0.03
            mls.process(*smoothed_cloud);

            // 6. Now, explicitly decide what to do with the result.
            // This is safer and makes your intent clear.
            aligned_cloud_ = smoothed_cloud;

            // 4. 提取边缘点
            edge_cloud_->clear();
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            normal_estimator.setInputCloud(aligned_cloud_);
            normal_estimator.setRadiusSearch(param["radius_search"]); // 设置法线估计的搜索半径
            normal_estimator.compute(*normals);
        
            pcl::PointCloud<pcl::Boundary> boundaries;
            pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
            boundary_estimator.setInputCloud(aligned_cloud_);
            boundary_estimator.setInputNormals(normals);
            boundary_estimator.setRadiusSearch(param["radius_search"]); // 设置边界检测的搜索半径
            boundary_estimator.setAngleThreshold(param["angle_threshold"]); // 设置角度阈值
            boundary_estimator.compute(boundaries);
        
            for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
                if (boundaries.points[i].boundary_point > 0) {
                    edge_cloud_->push_back(aligned_cloud_->points[i]);
                }
            }
            ROS_INFO("Extracted %ld edge points.", edge_cloud_->size());

            /* save the edge cloud for debugging */
            pcl::io::savePCDFileASCII("/home/ali/Parallels Shared Folders/sharedFolder/our_edge_cloud_" + std::to_string(trial_number) + ".pcd", *edge_cloud_);


            // 5. 对边缘点进行聚类
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
            tree2->setInputCloud(edge_cloud_);
        
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(param["cluster_tolerance"]); // 设置聚类距离阈值
            ec.setMinClusterSize(param["min_cluster_size"]);     // 最小点数
            ec.setMaxClusterSize(param["max_cluster_size"]);   // 最大点数
            ec.setSearchMethod(tree2);
            ec.setInputCloud(edge_cloud_);
            ec.extract(cluster_indices);
        
            ROS_INFO("Number of edge clusters: %ld", cluster_indices.size());
        
            // 6. 对每个聚类进行圆拟合
            center_z0_cloud_->reserve(4);
            Eigen::Matrix3d R_inv = R.inverse();
        
            // 对每个聚类进行圆拟合
            for (size_t i = 0; i < cluster_indices.size(); ++i) 
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& idx : cluster_indices[i].indices) {
                    cluster->push_back(edge_cloud_->points[idx]);
                }

                /* save the cluster for debugging */    
                pcl::io::savePCDFileASCII("/home/ali/Parallels Shared Folders/sharedFolder/trial_" + std::to_string(trial_number) + "_cluster_" + std::to_string(i) + ".pcd", *cluster);
        
                // 圆拟合
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_CIRCLE2D);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(param["distance_threshold"]); // 设置距离阈值
                seg.setMaxIterations(1000);     // 设置最大迭代次数
                seg.setInputCloud(cluster);
                seg.segment(*inliers, *coefficients);
        
                if (inliers->indices.size() > 0) 
                {
                    // 计算拟合误差
                    double error = 0.0;
                    for (const auto& idx : inliers->indices) 
                    {
                        double dx = cluster->points[idx].x - coefficients->values[0];
                        double dy = cluster->points[idx].y - coefficients->values[1];
                        double distance = sqrt(dx * dx + dy * dy) - circle_radius_; // 距离误差
                        error += abs(distance);
                    }
                    error /= inliers->indices.size();
        
                    // 如果拟合误差较小，则认为是一个圆洞
                    if (error < 0.02) 
                    {
                        // 将恢复后的圆心坐标添加到点云中
                        pcl::PointXYZ center_point;
                        center_point.x = coefficients->values[0];
                        center_point.y = coefficients->values[1];
                        center_point.z = 0.0;
                        center_z0_cloud_->push_back(center_point);

                        // 将圆心坐标逆变换回原始坐标系
                        Eigen::Vector3d aligned_point(center_point.x, center_point.y, center_point.z + average_z);
                        Eigen::Vector3d original_point = R_inv * aligned_point;

                        pcl::PointXYZ center_point_origin;
                        center_point_origin.x = original_point.x();
                        center_point_origin.y = original_point.y();
                        center_point_origin.z = original_point.z();
                        center_cloud->points.push_back(center_point_origin);
                    }
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const { return filtered_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlaneCloud() const { return plane_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const { return aligned_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEdgeCloud() const { return edge_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenterZ0Cloud() const { return center_z0_cloud_; }
    
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif