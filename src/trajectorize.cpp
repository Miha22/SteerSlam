#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include <gflags/gflags.h>

#include <System.h>
#include <io/frame_reader.hpp>
#include <slam/frame_processor.hpp>

extern "C" {
	#include <libavformat/avformat.h>
	#include <libswscale/swscale.h>
}

DEFINE_bool(enable_visualization, true, "Display video and 3D map during processing.");
DEFINE_bool(flip_vertical, false, "Flip input video frames vertically.");
DEFINE_bool(flip_horizontal, false, "Flip input video frames horizontally.");
DEFINE_bool(save_segment_videos, false, "Save videos for each successfully tracked SLAM segment.");
DEFINE_string(vocabular, "/path/to/ORBvoc.txt", "Path to the ORB vocabulary file.");
DEFINE_string(camera_config, "/path/to/calibration.xml", "Path to the camera settings (.yml) file.");
DEFINE_string(output_directory, "/path/to/output", "Directory for output trajectories and videos.");
DEFINE_string(input_video, "/path/to/input_video.mp4", "Path to the input video file.");


namespace {
	std::string GenerateTrajectoryFileName(const std::string &output_dir, int frame_id, const std::string &extension) {
		std::ostringstream filename;
		filename << output_dir << "/trajectory_segment_" << frame_id << "." << extension;
		return filename.str();
	}
}

int main(int argc, char **argv) {
	CHECK(!FLAGS_vocabular.empty()) << "Vocabulary file path is required.";
	CHECK(!FLAGS_camera_config.empty()) << "Camera settings file path is required.";
	CHECK(!FLAGS_input_video.empty()) << "Input video file path is required.";

	av_register_all();

	auto image_reader = slampilot::MakeImageSequenceSource(
			FLAGS_input_video, FLAGS_flip_vertical, FLAGS_flip_horizontal);
	LOG(INFO) << "Image sequence reader initialized.";

	auto vocabulary = std::make_unique<ORB_SLAM2::ORBVocabulary>(FLAGS_vocabulary_file);
	LOG(INFO) << "ORB vocabulary loaded.";

	int frame_id = 0;
	while (image_reader->hasNext()) {
			LOG(INFO) << "Starting SLAM processing for segment " << frame_id;

			auto slam_system = std::make_unique<ORB_SLAM2::System>(
			FLAGS_vocabulary_file, FLAGS_camera_settings, ORB_SLAM2::System::MONOCULAR, FLAGS_enable_visualization);

			std::string json = GenerateTrajectoryFileName(FLAGS_output_directory, frame_id, "json");
			std::unique_ptr<slampilot::ImageSequenceSink> videosink;

			if (FLAGS_save_segment_videos) {
					std::string trj_video = GenerateTrajectoryFileName(FLAGS_output_directory, frame_id, "mp4");
					videosink = std::make_unique<slampilot::ImageSequenceVideoFileSink>(trj_video, 30);
					LOG(INFO) << "Video sink created for segment " << frame_id;
			}

			slampilot::TrackImageSequence(slam_system.get(), *image_reader, json, videosink.get());

			slam_system->Shutdown();
			LOG(INFO) << "SLAM processing completed for segment " << frame_id;

			frame_id++;
	}

    LOG(INFO) << "Processing complete. Exiting.";

  return EXIT_SUCCESS;
}