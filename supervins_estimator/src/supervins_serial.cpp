/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <set>
#include <thread>
#include <mutex>
#include <memory>
#include <algorithm>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

//将ROS图像消息格式转化为OpenCV格式 cv::Mat
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat img = ptr->image.clone();

    if (true) {
        // rescale image
        cv::resize(img, img, {COL, ROW});
    }
    return img;
}

//IMU回调函数,imu_buf没用，直接把IMU数据往estimator里输入
void imu_callback(double time, const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    if (std::abs(time - t) >= 0.1) {
        std::cout << "timestamp mismatch.\n";
        std::exit(150);
    }
    estimator.inputIMU(t, acc, gyr);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 3)
    {
        printf("please intput: rosrun supervins serial <config_file> <bag_path> \n"
               "    for example: supervins supervins_node "
               "./config/euroc/euroc_mono_imu_config.yaml\n");
        return 1;
    }

    string config_file = argv[1];
    string bag_path = argv[2];
    printf("config_file: %s, bag_path: %s\n", argv[1], argv[2]);

    //读取参数，包括相机参数、IMU参数、特征参数等等，读入到parameters.cpp这个文件里的变量里
    // parameters, including camera parameters, imu parameters, feature parameters, etc., into variables in the file parameters.cpp
    readParameters(config_file);

    // 路径配置
    // Path configuration
    // std::string project_source_dir = PROJECT_SOURCE_DIR;
    // extractor_weight_global_path = project_source_dir + "/" + extractor_weight_relative_path;
    // matcher_weight_global_path = project_source_dir + "/" + matcher_weight_relative_path;
    // VINS_RESULT_PATH = project_source_dir + "/" + VINS_RESULT_PATH;

    //给estimator设置参数，因为一些参数可能被优化，所以可能会重置参数，注意，如果开启了多线程模式，在setParameter()中就已经将状态估计函数放入一个独立线程运行了
    //Set parameters for the estimator. Because some parameters may be optimized, the parameters may be reset. Note that if the multi-thread mode is turned on, the state estimation function has been put into an independent thread to run in set parameter().
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    registerPub(n);

    

    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    // rosbag::View view_full;
    rosbag::View view_full;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);

    double bag_true_start_secs = view_full.getBeginTime().toSec();
    double bag_true_end_secs = view_full.getEndTime().toSec();
    double bag_true_duration_secs = bag_true_end_secs - bag_true_start_secs;

    double bag_start_secs, bag_durr_secs, bag_end_secs;
    n.param<double>("bag_start", bag_start_secs, 0.0);
    n.param<double>("bag_durr", bag_durr_secs, -1.0);
    n.param<double>("bag_end", bag_end_secs, bag_true_duration_secs);
    ROS_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start_secs);
    ROS_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr_secs);
    ROS_DEBUG("[SERIAL]: bag end: %.1f\n", bag_end_secs);

    if (std::abs(bag_end_secs) < 0.0001) {
        bag_end_secs = bag_true_duration_secs;
    }

    if (bag_durr_secs > 0.0) {
        bag_end_secs = bag_start_secs + bag_durr_secs;
    }

    if (bag_end_secs < 0.0) {
        bag_end_secs += bag_true_duration_secs;
    }

    bag_durr_secs = bag_end_secs - bag_start_secs;

    rosbag::View view;

    view.addQuery(bag, view_full.getBeginTime() + ros::Duration(bag_start_secs), view_full.getBeginTime() + ros::Duration(bag_end_secs));

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    // view_full.addQuery(bag);
    // ros::Time time_init = view_full.getBeginTime();
    // time_init += ros::Duration(bag_start);
    // ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    // PRINT_DEBUG("time start = %.6f\n", time_init.toSec());
    // PRINT_DEBUG("time end   = %.6f\n", time_finish.toSec());
    // view.addQuery(bag, time_init, time_finish);

    if (view.size() == 0) {
        ROS_ERROR("[SERIAL]: No messages to play on specified topics.  Exiting.\n");
        ros::shutdown();
        return 1;
    }

    // We going to loop through and collect a list of all messages
    // This is done so we can access arbitrary points in the bag
    // NOTE: if we instantiate messages here, this requires the whole bag to be read
    // NOTE: thus we just check the topic which allows us to quickly loop through the index
    // NOTE: see this PR https://github.com/ros/ros_comm/issues/117
    double max_camera_time = -1;
    std::vector<std::unique_ptr<rosbag::MessageInstance>> msgs;
    std::vector<std::string> topic_cameras = {IMAGE0_TOPIC, IMAGE1_TOPIC};
    for (const rosbag::MessageInstance &msg : view) {
        if (!ros::ok())
            break;
        if (msg.getTopic() == IMU_TOPIC && USE_IMU) {
            // if (msg.instantiate<sensor_msgs::Imu>() == nullptr) {
            //   PRINT_ERROR(RED "[SERIAL]: IMU topic has unmatched message types!!\n" RESET);
            //   PRINT_ERROR(RED "[SERIAL]: Supports: sensor_msgs::Imu\n" RESET);
            //   return EXIT_FAILURE;
            // }
            msgs.push_back(std::move(std::make_unique<rosbag::MessageInstance>(msg)));
        }
        for (int i = 0; i < NUM_OF_CAM; i++) {
            if (msg.getTopic() == topic_cameras.at(i)) {
                // sensor_msgs::CompressedImage::ConstPtr img_c = msg.instantiate<sensor_msgs::CompressedImage>();
                // sensor_msgs::Image::ConstPtr img_i = msg.instantiate<sensor_msgs::Image>();
                // if (img_c == nullptr && img_i == nullptr) {
                //   PRINT_ERROR(RED "[SERIAL]: Image topic has unmatched message types!!\n" RESET);
                //   PRINT_ERROR(RED "[SERIAL]: Supports: sensor_msgs::Image and sensor_msgs::CompressedImage\n" RESET);
                //   return EXIT_FAILURE;
                // }
                msgs.push_back(std::move(std::make_unique<rosbag::MessageInstance>(msg)));
                max_camera_time = std::max(max_camera_time, msg.getTime().toSec());
            }
        }
    }
    ROS_DEBUG("[SERIAL]: total of %zu messages!\n", msgs.size());

    // sort the array
    std::sort(msgs.begin(), msgs.end(), [](const auto& a, const auto& b) -> bool {
        return a->getTime().toSec() < b->getTime().toSec();
    });

    ROS_DEBUG("[SERIAL]: array sorted!\n");

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Loop through our message array, and lets process them
    std::set<int> used_index;
    double prev_it_ts = ros::WallTime::now().toSec();
    for (int m = 0; m < (int)msgs.size(); m++) {
        ros::spinOnce();
        // End once we reach the last time, or skip if before beginning time (shouldn't happen)
        if (!ros::ok() /* || msgs.at(m)->getTime() > time_finish */ || msgs.at(m)->getTime().toSec() > max_camera_time)
            break;
        // if (msgs.at(m)->getTime() < time_init)
        //     continue;

        // Skip messages that we have already used
        if (used_index.find(m) != used_index.end()) {
            used_index.erase(m);
            prev_it_ts = ros::WallTime::now().toSec();
            continue;
        }

        // IMU processing
        if (msgs.at(m)->getTopic() == IMU_TOPIC && USE_IMU) {
            // PRINT_DEBUG("processing imu = %.3f sec\n", msgs.at(m).getTime().toSec() - time_init.toSec());
            imu_callback(msgs.at(m)->getTime().toSec(), msgs.at(m)->instantiate<sensor_msgs::Imu>());
        }

        // Camera processing
        for (int cam_id = 0; cam_id < NUM_OF_CAM; cam_id++) {
            // Skip if this message is not a camera topic
            if (msgs.at(m)->getTopic() != topic_cameras.at(cam_id))
                continue;

            // We have a matching camera topic here, now find the other cameras for this time
            // For each camera, we will find the nearest timestamp (within 0.02sec) that is greater than the current
            // If we are unable, then this message should just be skipped since it isn't a sync'ed pair!
            std::map<int, int> camid_to_msg_index;
            double meas_time = msgs.at(m)->getTime().toSec();
            for (int cam_idt = 0; cam_idt < NUM_OF_CAM; cam_idt++) {
                if (cam_idt == cam_id) {
                    camid_to_msg_index.insert({cam_id, m});
                    continue;
                }
                int cam_idt_idx = -1;
                for (int mt = m; mt < (int)msgs.size(); mt++) {
                    if (msgs.at(mt)->getTopic() != topic_cameras.at(cam_idt))
                        continue;
                    if (std::abs(msgs.at(mt)->getTime().toSec() - meas_time) < 0.02)
                        cam_idt_idx = mt;
                    break;
                }
                if (cam_idt_idx != -1) {
                    camid_to_msg_index.insert({cam_idt, cam_idt_idx});
                }
            }

            // Skip processing if we were unable to find any messages
            if ((int)camid_to_msg_index.size() != NUM_OF_CAM) {
                ROS_DEBUG("[SERIAL]: Unable to find stereo pair for message %d at %.2f into bag (will skip!)\n", m,
                            meas_time /* - time_init.toSec() */);
                continue;
            }

            // Check if we should initialize using the groundtruth
            // Eigen::Matrix<double, 17, 1> imustate;
            // if (!gt_states.empty() && !sys->initialized() && ov_core::DatasetReader::get_gt_state(meas_time, imustate, gt_states)) {
            //     // biases are pretty bad normally, so zero them
            //     // imustate.block(11,0,6,1).setZero();
            //     sys->initialize_with_gt(imustate);
            // }

            // Pass our data into our visualizer callbacks!
            // PRINT_DEBUG("processing cam = %.3f sec\n", msgs.at(m).getTime().toSec() - time_init.toSec());
            double current_msg_ts = msgs.at(m)->getTime().toSec();
            ROS_INFO_STREAM("Bag time " << (current_msg_ts - bag_true_start_secs) << " / " << bag_true_duration_secs
                << ", progress " << (current_msg_ts - bag_true_start_secs - bag_start_secs) << "/" << bag_durr_secs
            );
            if (NUM_OF_CAM == 1) {
                auto msg0 = *msgs.at(camid_to_msg_index.at(0));
                const auto& img0 = msg0.instantiate<sensor_msgs::Image>();
                auto time0 = img0->header.stamp.toSec();
                used_index.insert(camid_to_msg_index.at(0));
                cv::Mat mat0 = getImageFromMsg(img0);
                estimator.inputImage(time0, mat0);
            } else if (NUM_OF_CAM == 2) {
                auto msg0 = *msgs.at(camid_to_msg_index.at(0));
                auto msg1 = *msgs.at(camid_to_msg_index.at(1));
                const auto& img0 = msg0.instantiate<sensor_msgs::Image>();
                const auto& img1 = msg1.instantiate<sensor_msgs::Image>();
                auto time0 = img0->header.stamp.toSec();
                used_index.insert(camid_to_msg_index.at(0));
                used_index.insert(camid_to_msg_index.at(1));
                cv::Mat mat0 = getImageFromMsg(img0);
                cv::Mat mat1 = getImageFromMsg(img1);
                estimator.inputImage(time0, mat0, mat1);
            } else {
                ROS_ERROR("[SERIAL]: We currently only support 1 or 2 camera serial input....\n");
                return 2;
            }
            double it_ts = ros::WallTime::now().toSec();
            ROS_INFO_STREAM("Image processing took " << (it_ts - prev_it_ts) << " seconds (" << (1/(it_ts - prev_it_ts)) << " hz)");
            prev_it_ts = it_ts;
            break;
        }
        
    }

    ROS_INFO_STREAM("Finished serial processing, exiting.");
    return 0;
}
