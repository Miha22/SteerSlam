#ifndef SLAMPILOT_FRAME_PROCESSOR_HPP
#define SLAMPILOT_FRAME_PROCESSOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <json.hpp>
#include <ORB_SLAM2/System.h>

#include <io/frame_reader.hpp>
#include <io/frame_writer.hpp>

namespace slampilot {

    nlohmann::json PoseToJson(const ORB_SLAM2::Pose &pose);

    std::unique_ptr<cv::PCA> ComputePCA(const std::vector<ORB_SLAM2::PoseWithTimestamp> &traj);

    std::unique_ptr<std::vector<cv::Mat>> ProjectVectors(
        const std::vector<ORB_SLAM2::PoseWithTimestamp> &traj, const cv::Mat &plane);

    std::vector<double> ComputeTurnAngles(const std::vector<cv::Mat> &dirs);

    void AddPlaneToJson(nlohmann::json *root, const cv::Mat &plane);

    void AddTrajectoryToJson(nlohmann::json *root, 
                            const std::vector<ORB_SLAM2::PoseWithTimestamp> &traj,
                            const std::vector<cv::Mat> &proj_dirs, 
                            const std::vector<double> &angles, 
                            int frame_offset);

    bool TrackImageSequence(ORB_SLAM2::System *slam, 
                            ImageSequenceSource &src, 
                            const std::string &output_file, 
                            ImageSequenceSink *sink);

    }

#endif