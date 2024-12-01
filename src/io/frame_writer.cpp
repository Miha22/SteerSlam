#include <io/frame_writer.hpp>
namespace slampilot {

    ImageSequenceVideoFileSink::ImageSequenceVideoFileSink(const std::string &output_file, int frame_rate)
        : output_file_(output_file), frame_rate_(frame_rate) {}

    ImageSequenceVideoFileSink::~ImageSequenceVideoFileSink() {
        if (frame_out_) {
            av_write_trailer(format_ctx_);
            avcodec_close(stream_->codec);
            av_frame_free(&frame_out_);
            av_frame_free(&frame_rgb_);
            sws_freeContext(sws_ctx_);
            avio_closep(&format_ctx_->pb);
            avformat_free_context(format_ctx_);
        }
    }

    void ImageSequenceVideoFileSink::InitStream(int height, int width) {
        CHECK(frame_out_ == nullptr);

        LOG(INFO) << "Initializing video stream for: " << output_file_;

        frame_out_ = CHECK_NOTNULL(av_frame_alloc());
        frame_rgb_ = CHECK_NOTNULL(av_frame_alloc());

        avformat_alloc_output_context2(&format_ctx_, nullptr, nullptr, output_file_.c_str());
        CHECK_NOTNULL(format_ctx_);

        format_ctx_->oformat->video_codec = AV_CODEC_ID_H264;

        encoder_ = CHECK_NOTNULL(avcodec_find_encoder(format_ctx_->oformat->video_codec));
        stream_ = CHECK_NOTNULL(avformat_new_stream(format_ctx_, encoder_));
        stream_->id = format_ctx_->nb_streams - 1;

        codec_ctx_ = stream_->codec;
        codec_ctx_->codec_id = format_ctx_->oformat->video_codec;
        codec_ctx_->bit_rate = 4000000;
        codec_ctx_->width = width;
        codec_ctx_->height = height;

        stream_->time_base = {1, frame_rate_};
        codec_ctx_->time_base = stream_->time_base;
        codec_ctx_->gop_size = 12;
        codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;

        if (format_ctx_->oformat->flags & AVFMT_GLOBALHEADER) {
            codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        CHECK_GE(avcodec_open2(codec_ctx_, encoder_, nullptr), 0);
        av_dump_format(format_ctx_, 0, output_file_.c_str(), 1);

        CHECK_GE(avio_open(&format_ctx_->pb, output_file_.c_str(), AVIO_FLAG_WRITE), 0);
        CHECK_GE(avformat_write_header(format_ctx_, nullptr), 0);

        int rgb_buffer_size = avpicture_get_size(AV_PIX_FMT_RGB24, width, height);
        uint8_t *rgb_buffer = (uint8_t *)av_malloc(rgb_buffer_size * sizeof(uint8_t));
        CHECK_GE(avpicture_fill((AVPicture *)frame_rgb_, rgb_buffer, AV_PIX_FMT_RGB24, width, height), 0);
        frame_rgb_->height = height;
        frame_rgb_->width = width;

        int out_buffer_size = avpicture_get_size(codec_ctx_->pix_fmt, width, height);
        uint8_t *out_buffer = (uint8_t *)av_malloc(out_buffer_size * sizeof(uint8_t));
        CHECK_GE(avpicture_fill((AVPicture *)frame_out_, out_buffer, codec_ctx_->pix_fmt, width, height), 0);
        frame_out_->height = height;
        frame_out_->width = width;

        sws_ctx_ = sws_getCachedContext(nullptr, width, height, AV_PIX_FMT_RGB24, width, height,
                                        codec_ctx_->pix_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
        CHECK_NOTNULL(sws_ctx_);
    }

    void ImageSequenceVideoFileSink::consume(const cv::Mat &frame) {
        if (frame_out_ == nullptr) {
            InitStream(frame.rows, frame.cols);
        }

        CHECK_EQ(frame.rows, frame_rgb_->height);
        CHECK_EQ(frame.cols, frame_rgb_->width);

        LOG(INFO) << "Processing frame " << next_pts_;

        for (int y = 0; y < frame.rows; ++y) {
            for (int x = 0; x < frame.cols; ++x) {
                const cv::Vec3b &pixel = frame.at<cv::Vec3b>(y, x);
                unsigned char *dest = frame_rgb_->data[0] + y * frame_rgb_->linesize[0] + 3 * x;
                dest[0] = pixel[0];
                dest[1] = pixel[1];
                dest[2] = pixel[2];
            }
        }

        sws_scale(sws_ctx_, frame_rgb_->data, frame_rgb_->linesize, 0, frame.rows,
                  frame_out_->data, frame_out_->linesize);
        frame_out_->pts = next_pts_++;

        AVPacket pkt = {0};
        av_init_packet(&pkt);
        int got_packet = 0;
        CHECK_GE(avcodec_encode_video2(stream_->codec, &pkt, frame_out_, &got_packet), 0);
        if (got_packet) {
            av_packet_rescale_ts(&pkt, stream_->codec->time_base, stream_->time_base);
            pkt.stream_index = stream_->index;
            CHECK_GE(av_interleaved_write_frame(format_ctx_, &pkt), 0);
        }
    }
}