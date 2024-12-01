#include <fstream>
#include <iostream>
#include <io/frame_reader.hpp>

namespace slampilot {

namespace {
    void ReadLineOrDie(std::istream *input_stream, std::string *line) {
        CHECK_NOTNULL(input_stream);
        CHECK_NOTNULL(line);
        CHECK(!input_stream->eof());
        getline(*input_stream, *line);
    }
} 

std::vector<TimestampedImageName> LoadImages(const std::string &file_path) {
    std::ifstream file(file_path.c_str());
    CHECK(file.is_open()) << "Failed to open file: " << file_path;

    std::string line;
    ReadLineOrDie(&file, &line);
    ReadLineOrDie(&file, &line);
    ReadLineOrDie(&file, &line);

    std::vector<TimestampedImageName> images;
    while (!file.eof()) {
        ReadLineOrDie(&file, &line);
        if (!line.empty()) {
            std::stringstream line_stream(line);
            TimestampedImageName image_entry;
            line_stream >> image_entry.timestamp;
            CHECK(!line_stream.eof());
            line_stream >> image_entry.imageName;
            images.push_back(image_entry);
        }
    }
    return images;
}

FlippedImageSequenceSource::FlippedImageSequenceSource(
    std::unique_ptr<ImageSequenceSource> src, int flip_axis)
    : source_(std::move(src)), flip_axis_(flip_axis) {}

bool FlippedImageSequenceSource::hasNext() {
    return source_->hasNext();
}

ORB_SLAM2::TimestampedImage FlippedImageSequenceSource::next() {
    ORB_SLAM2::TimestampedImage img = source_->next();
    cv::Mat flipped;
    cv::flip(img.image, flipped, flip_axis_);
    img.image = flipped;
    return img;
}

namespace {
    int FindVideoStream(const AVFormatContext &format_ctx) {
        for (unsigned int i = 0; i < format_ctx.nb_streams; ++i) {
            if (format_ctx.streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
                return i;
            }
        }
        CHECK(false) << "No video stream found in the format context.";
        return -1; 
    }
}

VideoImageSequenceSource::VideoImageSequenceSource(const std::string &file_path)
    : format_ctx_(avformat_alloc_context()),
      frame_src_(av_frame_alloc()), frame_rgb_(av_frame_alloc()) {

    CHECK_EQ(avformat_open_input(&format_ctx_, file_path.c_str(), nullptr, nullptr), 0)
        << "Failed to open video file: " << file_path;
    CHECK_GE(avformat_find_stream_info(format_ctx_, nullptr), 0)
        << "Failed to retrieve stream info.";

    av_dump_format(format_ctx_, 0, file_path.c_str(), 0);

    video_stream_idx_ = FindVideoStream(*format_ctx_);
    CHECK_GE(video_stream_idx_, 0);

    codec_ctx_ = CHECK_NOTNULL(format_ctx_->streams[video_stream_idx_]->codec);
    codec_ = CHECK_NOTNULL(avcodec_find_decoder(codec_ctx_->codec_id));
    CHECK_GE(avcodec_open2(codec_ctx_, codec_, nullptr), 0);

    int rgb_size = avpicture_get_size(AV_PIX_FMT_RGB24, codec_ctx_->width, codec_ctx_->height);
    uint8_t *rgb_buffer = (uint8_t *)av_malloc(rgb_size * sizeof(uint8_t));
    avpicture_fill((AVPicture *)frame_rgb_, rgb_buffer, AV_PIX_FMT_RGB24,
                   codec_ctx_->width, codec_ctx_->height);

    next_frame_.image = cv::Mat(codec_ctx_->height, codec_ctx_->width, CV_8UC3);

    sws_ctx_ = sws_getCachedContext(nullptr, codec_ctx_->width, codec_ctx_->height,
                                    codec_ctx_->pix_fmt, codec_ctx_->width,
                                    codec_ctx_->height, AV_PIX_FMT_RGB24,
                                    SWS_BILINEAR, nullptr, nullptr, nullptr);

    FetchNextFrame();
}

VideoImageSequenceSource::~VideoImageSequenceSource() {
    av_free(frame_rgb_);
    av_free(frame_src_);
    avcodec_close(codec_ctx_);
    avformat_close_input(&format_ctx_);
}

bool VideoImageSequenceSource::hasNext() {
    return has_next_;
}

ORB_SLAM2::TimestampedImage VideoImageSequenceSource::next() {
    CHECK(has_next_);
    ORB_SLAM2::TimestampedImage img = next_frame_;
    FetchNextFrame();
    return img;
}

void VideoImageSequenceSource::FetchNextFrame() {
    has_next_ = false;
    AVPacket packet;
    while (av_read_frame(format_ctx_, &packet) >= 0) {
        if (packet.stream_index != video_stream_idx_) {
            av_packet_unref(&packet);
            continue;
        }

        int got_picture = 0;
        avcodec_decode_video2(codec_ctx_, frame_src_, &got_picture, &packet);
        av_packet_unref(&packet);
        if (!got_picture) {
            continue;
        }

        sws_scale(sws_ctx_, frame_src_->data, frame_src_->linesize, 0,
                  frame_src_->height, frame_rgb_->data, frame_rgb_->linesize);

        for (int row = 0; row < frame_src_->height; ++row) {
            for (int col = 0; col < frame_src_->width; ++col) {
                cv::Vec3b &dest = next_frame_.image.at<cv::Vec3b>(row, col);
                const unsigned char *src = frame_rgb_->data[0] +
                                           row * frame_rgb_->linesize[0] + 3 * col;
                dest[0] = src[0];
                dest[1] = src[1];
                dest[2] = src[2];
            }
        }

        next_frame_.timestamp =
            av_frame_get_best_effort_timestamp(frame_src_) *
            av_q2d(format_ctx_->streams[video_stream_idx_]->time_base);
        ++next_frame_.frame_id;
        has_next_ = true;
        break;
    }
}

namespace {
    int DetermineFlipAxis(bool flip_vert, bool flip_horiz) {
        if (flip_vert && !flip_horiz) {
            return 0;
        } else if (!flip_vert && flip_horiz) {
            return 1;
        } else if (flip_vert && flip_horiz) {
            return -1;
        } else {
            CHECK(false) << "At least one of (flip_vert, flip_horiz) must be true.";
            return -1;
        }
    }
}

  std::unique_ptr<ImageSequenceSource>
  MakeImageSequenceSource(const std::string &file_path, bool flip_vert, bool flip_horiz) {
      auto base_source = std::make_unique<VideoImageSequenceSource>(file_path);
      if (!flip_vert && !flip_horiz) {
          return base_source;
      }

      int flip_axis = DetermineFlipAxis(flip_vert, flip_horiz);
      return std::make_unique<FlippedImageSequenceSource>(std::move(base_source), flip_axis);
  }
} 