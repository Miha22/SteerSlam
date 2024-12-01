#include <Eigen/Geometry>
#include <slam/frame_processor.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <json.hpp>

namespace slampilot {
    namespace {

        nlohmann::json PoseToJson(const ORB_SLAM2::Pose &pose) {
            nlohmann::json result;
            result[kTranslation] = {pose.translation(0), pose.translation(1), pose.translation(2)};
            
            nlohmann::json rotation;
            rotation[kW] = pose.rotation_raw.at(3);
            rotation[kX] = pose.rotation_raw.at(0);
            rotation[kY] = pose.rotation_raw.at(1);
            rotation[kZ] = pose.rotation_raw.at(2);
            result[kRotation] = rotation;

            return result;
        }

        std::unique_ptr<cv::PCA> ComputePCA(const std::vector<ORB_SLAM2::PoseWithTimestamp> &traj) {
            cv::Mat traj_matrix(3, traj.size(), CV_64FC1);
            for (size_t idx = 0; idx < traj.size(); ++idx) {
                const cv::Vec3d &trans = traj.at(idx).pose.translation;
                traj_matrix.at<double>(0, idx) = trans(0);
                traj_matrix.at<double>(1, idx) = trans(1);
                traj_matrix.at<double>(2, idx) = trans(2);
            }
            return std::make_unique<cv::PCA>(traj_matrix, cv::noArray(), CV_PCA_DATA_AS_COL);
        }

        std::unique_ptr<std::vector<cv::Mat>> ProjectVectors(
            const std::vector<ORB_SLAM2::PoseWithTimestamp> &traj, const cv::Mat &plane) {
            
            const Eigen::Vector3d z_axis(0, 0, 1);
            auto proj_dirs = std::make_unique<std::vector<cv::Mat>>();

            for (const auto &point : traj) {
                Eigen::Vector3d cam_dir = Eigen::Quaterniond(point.pose.matrix3d)._transformVector(z_axis);
                proj_dirs->emplace_back(plane * cv::Mat(cv::Vec3d(cam_dir(0), cam_dir(1), cam_dir(2))));
                CHECK_EQ(proj_dirs->back().rows, 2);
                CHECK_EQ(proj_dirs->back().cols, 1);
            }
            return proj_dirs;
        }

        std::vector<double> ComputeTurnAngles(const std::vector<cv::Mat> &dirs) {
            std::vector<double> angles(dirs.size(), 0);

            for (size_t idx = 1; idx < dirs.size(); ++idx) {
                const cv::Mat &prev = dirs[idx - 1];
                const cv::Mat &curr = dirs[idx];
                const cv::Vec2d p(prev.at<double>(0, 0), prev.at<double>(0, 1));
                const cv::Vec2d c(curr.at<double>(0, 0), curr.at<double>(0, 1));

                double dot_prod = p.dot(c) / (cv::norm(p) * cv::norm(c));
                cv::Vec3d cross_prod = cv::Vec3d(p(0), p(1), 0).cross(cv::Vec3d(c(0), c(1), 0));
                angles[idx] = acos(dot_prod) * (cross_prod(2) > 0 ? 1.0 : -1.0);
            }
            return angles;
        }

        void AddPlaneToJson(nlohmann::json *root, const cv::Mat &plane) {
            CHECK_NOTNULL(root);
            CHECK_EQ(plane.rows, 2);
            CHECK_EQ(plane.cols, 3);

            (*root)[kPlane] = {
                {plane.at<double>(0, 0), plane.at<double>(0, 1), plane.at<double>(0, 2)},
                {plane.at<double>(1, 0), plane.at<double>(1, 1), plane.at<double>(1, 2)}
            };
        }

        void AddTrajectoryToJson(nlohmann::json *root, const std::vector<ORB_SLAM2::PoseWithTimestamp> &traj,
                                 const std::vector<cv::Mat> &proj_dirs, const std::vector<double> &angles,
                                 int frame_offset) {
            CHECK_NOTNULL(root);
            CHECK_EQ(traj.size(), proj_dirs.size());
            CHECK_EQ(traj.size(), angles.size());

            (*root)[kTrajectory] = {};

            for (size_t idx = 0; idx < traj.size(); ++idx) {
                if (traj[idx].pose.rotation_raw.size() != 4) continue;

                const auto &point = traj[idx];
                nlohmann::json pt_json;

                pt_json[kTimeUsec] = point.time_usec;
                pt_json[kIsLost] = point.is_lost;
                pt_json[kFrameId] = point.frame_id - frame_offset;
                pt_json[kPose] = PoseToJson(point.pose);

                const cv::Mat &dir = proj_dirs[idx];
                pt_json[kPlanarDirection] = {dir.at<double>(0, 0), dir.at<double>(0, 1)};
                pt_json[kTurnAngle] = angles[idx];

                (*root)[kTrajectory].push_back(pt_json);
            }
        }
    }

    bool TrackImageSequence(ORB_SLAM2::System *slam, ImageSequenceSource &src, const std::string &output_file,
                            ImageSequenceSink *sink) {
        CHECK_NOTNULL(slam);

        ORB_SLAM2::TimestampedImage img;
        int track_count = 0;
        int first_frame_id = 0;

        while (src.hasNext()) {
            img = src.next();
            CHECK(!img.image.empty());

            slam->TrackMonocular(img);
            auto state = slam->tracker().state();

            if (state == ORB_SLAM2::LOST) {
                LOG(WARNING) << "Tracking lost.";
                continue;
            }

            if (track_count == 0 && state == ORB_SLAM2::OK) {
                first_frame_id = img.frame_id;
            }

            track_count += (state == ORB_SLAM2::OK);

            if (sink) {
                sink->consume(img.image);
            }
        }

        const auto traj = slam->GetTrajectory();
        if (traj.empty()) {
            LOG(WARNING) << "Empty trajectory.";
            return false;
        }

        auto pca = ComputePCA(traj);
        LOG(INFO) << "PCA eigenvectors: " << pca->eigenvectors;
        LOG(INFO) << "PCA eigenvalues: " << pca->eigenvalues;

        if (pca->eigenvalues.at<double>(2) > pca->eigenvalues.at<double>(1) * 1e-2) {
            LOG(WARNING) << "Significant vertical motion detected. Dropping trajectory.";
            return false;
        }

        auto proj_dirs = ProjectVectors(traj, pca->eigenvectors.rowRange(0, 2));
        auto angles = ComputeTurnAngles(*proj_dirs);

        nlohmann::json output_json;
        AddPlaneToJson(&output_json, pca->eigenvectors.rowRange(0, 2));
        AddTrajectoryToJson(&output_json, traj, *proj_dirs, angles, sink ? first_frame_id : 0);

        std::ofstream out_stream(output_file);
        out_stream << output_json.dump(2) << std::endl;

        return true;
    }
}