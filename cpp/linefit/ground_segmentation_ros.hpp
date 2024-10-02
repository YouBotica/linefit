#pragma once
#include <mutex>

#include "segment.h"
#include "mics.h"
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
#include <algorithm>
#include <iostream>
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include "toml.hpp"

struct GroundSegmentationParams {
    double sensor_height;
    double r_min_square;
    double r_max_square;
    int n_bins;
    int n_segments;
    double min_slope;
    double max_slope;
    double max_dist_to_line;
    double max_error_square;
    double long_threshold;
    double max_long_height;
    double max_start_height;
    double line_search_angle;
    unsigned int n_threads;
  };

// typedef std::vector<Eigen::Vector3d> PointCloud;

// typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> PointLine;

class GroundSegmentationRos {   
public:

  GroundSegmentationRos(){};
  // GroundSegmentationRos(rclcpp::Node *nh): ros_node(nh);
  GroundSegmentationRos(rclcpp::Node *nh);

  std::vector<bool> segment(PointCloud &cloud);

  // std::vector<bool> segment(const PointCloud &cloud);

private:
    bool verbose_ = false;
    GroundSegmentationParams params_;

    double r_min;
    double r_max;

    typedef std::vector<Eigen::Vector3d> PointCloud;

    typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> PointLine;

    // Access with segments_[segment][bin].
    std::vector<Segment> segments_;

    // Bin index of every point.
    std::vector<std::pair<int, int> > bin_index_;

    // 2D coordinates (d, z) of every point in its respective segment.
    std::vector<Bin::MinZPoint> segment_coordinates_;

    void assignCluster(std::vector<bool>* segmentation);

    void assignClusterThread(const unsigned int& start_index,
                            const unsigned int& end_index,
                            std::vector<bool>* segmentation);

    void insertPoints(const PointCloud& cloud);

    void insertionThread(const PointCloud& cloud,
                        const size_t start_index,
                        const size_t end_index);


    void getLines();

    void lineFitThread(const unsigned int start_index, const unsigned int end_index);

    Eigen::Vector3d minZPointTo3d(const Bin::MinZPoint& min_z_point, const double& angle);

    void getMinZPointCloud(PointCloud* cloud);

    void resetSegments();
    
    rclcpp::Node *ros_node;

};
