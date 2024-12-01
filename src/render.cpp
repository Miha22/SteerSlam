#include <fstream>
#include <iostream>
#include <gflags/gflags.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <json.hpp>
#include <io/frame_reader.hpp>
#include <io/frame_writer.hpp>

DEFINE_double(scale, 6000.0, "Scale factor for converting angular velocity to rotation angle in degrees.");
DEFINE_double(smooth, 0.35, "Temporal smoothing parameter for rendering the steering wheel. Range: 0 to 1.");

DEFINE_string(json, "/path/to/trajectory.json", "JSON file containing the trajectory data.");
DEFINE_string(video_out, "/path/to/video_out.mp4", "Path to save the rendered output video.");
DEFINE_string(video_in, "/path/to/video_in.mp4", "Path to the input video file.");
DEFINE_string(wheel_pic, "/path/to/wheel_pic.jpeg", "Path to the steering wheel image.");
DEFINE_bool(vflip, false, "Flip input frames vertically.");
DEFINE_bool(hflip, false, "Flip input frames horizontally.");


int main(int argc, char **argv) {
	CHECK(!FLAGS_video_in.empty()) << "Input video file path is required.";
    CHECK(!FLAGS_json.empty()) << "Trajectory JSON file path is required.";
    CHECK(!FLAGS_wheel_pic.empty()) << "Steering wheel image file path is required.";

    av_register_all();

	auto image_reader = slampilot::MakeImageSequenceSource(
        FLAGS_video_in, FLAGS_vflip, FLAGS_hflip);
    LOG(INFO) << "Initialized image sequence reader for video: " << FLAGS_video_in;

    cv::Mat steering_wheel = cv::imread(FLAGS_wheel_pic, cv::IMREAD_COLOR);
    CHECK(!steering_wheel.empty()) << "Failed to load steering wheel pic.";
    cv::Mat steering_wheel;
    cv::cvtColor(steering_wheel, steering_wheel, cv::COLOR_BGR2RGB);
    LOG(INFO) << "Loaded steering wheel pic.";

	std::ifstream trajectories(FLAGS_json);
    nlohmann::json json;
    trajectories >> json;
    const auto &trajectory = json[slampilot::kTrajectory];
    LOG(INFO) << "Loaded trajectory data json: " << FLAGS_json;

    slampilot::ImageSequenceVideoFileSink video_sink(FLAGS_video_out, 30);

	ORB_SLAM2::TimestampedImage frame_current = image_reader->next();
	cv::Mat combined_frame(frame_current.image.rows + steering_wheel.rows,
						std::max(frame_current.image.cols, steering_wheel.cols),
						CV_8UC3);
	cv::Mat video_region = combined_frame.rowRange(0, frame_current.image.rows)
										.colRange(0, frame_current.image.cols);
	cv::Mat steering_region = combined_frame.rowRange(frame_current.image.rows,
													frame_current.image.rows + steering_wheel.rows)
											.colRange(0, steering_wheel.cols);

    double turnangle_smooth = 0.0;

    auto tr_entry = trajectory.begin();
    while (tr_entry != trajectory.end() && image_reader->hasNext()) {
        frame_current = image_reader->next();
        const int64_t frame_id_from_json = (*tr_entry)[slampilot::kFrameId];

        if (frame_current.frame_id < frame_id_from_json) {
            continue;
        }

        CHECK_EQ(frame_current.frame_id, frame_id_from_json)
            << "Frame ID mismatch: video frame ID and trajectory frame ID do not align.";

        double turn_angle = (*tr_entry)[slampilot::kTurnAngle];
        turnangle_smooth = (1.0 - FLAGS_smooth) * turnangle_smooth +
                              FLAGS_smooth * turn_angle;
        LOG(INFO) << "Processing frame " << frame_id_from_json
                  << " with smoothed turn angle: " << turnangle_smooth;

        frame_current.image.copyTo(video_region);

        cv::Mat rotation_matrix = cv::getRotationMatrix2D(
            cv::Point2f(steering_wheel.cols / 2, steering_wheel.rows / 2),
            turnangle_smooth * FLAGS_scale, 1.0);
        cv::warpAffine(steering_wheel, steering_region, rotation_matrix,
                       steering_wheel.size(), cv::INTER_LINEAR);

        video_sink.consume(combined_frame);

        ++tr_entry;
    }

    LOG(INFO) << "Video rendering complete, saved to: " << FLAGS_video_out;
    return EXIT_SUCCESS;
}