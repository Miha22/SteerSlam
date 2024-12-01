#ifndef SLAMPILOT_FRAME_WRITER_HPP
#define SLAMPILOT_FRAME_WRITER_HPP

#include <string>

extern "C" {
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
}

namespace slampilot {
    class ImageSequenceVideoFileSink {
    public:
        ImageSequenceVideoFileSink(const std::string &output_file, int frame_rate);
        ~ImageSequenceVideoFileSink();

        void consume(const cv::Mat &frame);

    private:
        void InitStream(int height, int width);

        std::string output_file_;
        int frame_rate_;
        int64_t next_pts_ = 0;

        AVFormatContext *format_ctx_ = nullptr;
        AVCodecContext *codec_ctx_ = nullptr;
        const AVCodec *encoder_ = nullptr;
        AVStream *stream_ = nullptr;
        AVFrame *frame_out_ = nullptr;
        AVFrame *frame_rgb_ = nullptr;
        SwsContext *sws_ctx_ = nullptr;
    };

}
#endif 