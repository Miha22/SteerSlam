#ifndef SLAMPILOT_FRAME_READER_HPP
#define SLAMPILOT_FRAME_READER_HPP

#include <string>
#include <vector>
#include <System.h>
#include <opencv2/highgui/highgui.hpp>

extern "C" {
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
}

namespace slampilot {
    struct TimestampedImageName {
        double timestamp;
        std::string imageName;
    };

    std::vector<TimestampedImageName> LoadImages(const std::string &file_path);

    class ImageSequenceSource {
    public:
        virtual ~ImageSequenceSource() = default;
        virtual bool hasNext() = 0;
        virtual ORB_SLAM2::TimestampedImage next() = 0;
    };

    class FlippedImageSequenceSource : public ImageSequenceSource {
    public:
        FlippedImageSequenceSource(std::unique_ptr<ImageSequenceSource> src, int flip_axis);
        bool hasNext() override;
        ORB_SLAM2::TimestampedImage next() override;

    private:
        std::unique_ptr<ImageSequenceSource> source_;
        int flip_axis_;
    };

    class VideoImageSequenceSource : public ImageSequenceSource {
    public:
        explicit VideoImageSequenceSource(const std::string &file_path);
        ~VideoImageSequenceSource();

        bool hasNext() override;
        ORB_SLAM2::TimestampedImage next() override;

    private:
        void FetchNextFrame();

        AVFormatContext *format_ctx_; 
        AVCodecContext *codec_ctx_;
        const AVCodec *codec_; 
        AVFrame *frame_src_;
        AVFrame *frame_rgb_;
        SwsContext *sws_ctx_;
        int video_stream_idx_;

        ORB_SLAM2::TimestampedImage next_frame_; 
        bool has_next_ = false; 
    };

    std::unique_ptr<ImageSequenceSource>
    MakeImageSequenceSource(const std::string &file_path, bool flip_vert, bool flip_horiz);

}

#endif
