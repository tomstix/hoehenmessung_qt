#include "realsense.h"

void RealsenseWorker::setResolution(Resolution res_)
{
    res = res_;
    switch (res)
    {
    case RES_424_240:
        m_width = 424;
        m_height = 240;
        break;
    case RES_480_270:
        m_width = 480;
        m_height = 270;
        break;
    case RES_640_360:
        m_width = 640;
        m_height = 360;
        break;
    case RES_640_480:
        m_width = 640;
        m_height = 480;
        break;
    case RES_1280_720:
        m_width = 1280;
        m_height = 720;
        break;
    default:
        break;
    }
    qDebug() << "Resolution set to " << m_width << "x" << m_height;
    emit widthChanged(m_width);
    emit heightChanged(m_height);
    emit resolutionChanged();
}
RealsenseWorker::Resolution RealsenseWorker::resolution() const
{
    return res;
}
int RealsenseWorker::width() const
{
    return m_width;
}
int RealsenseWorker::height() const
{
    return m_height;
}
void RealsenseWorker::stop()
{
    isRunning = false;
}

void RealsenseWorker::run()
{
    qDebug() << "Starting Realsense with " << m_width << "x" << m_height;
    cfg.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_RGB8, 30);
    pipe_profile = pipe.start(cfg);
    auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    auto depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsics = depth_stream.get_intrinsics();
    std::vector<std::vector<double>> mat{{intrinsics.fx, 0, intrinsics.ppx},
                                         {0, intrinsics.fy, intrinsics.ppy},
                                         {0, 0, 1}};
    intrinsic_matrix = mat;
    isRunning = true;
    while (isRunning)
    {
        frames = pipe.wait_for_frames();
        
        auto color_frame = frames.get_color_frame();

        auto vf = color_frame.as<rs2::video_frame>();
        if (color_frame.get_profile().format() == RS2_FORMAT_RGB8)
        {
            image = QImage((uchar *)color_frame.get_data(), m_width, m_height, m_width * 3, QImage::Format_RGB888);
        }
        emit colorImageReady();
    }
    qDebug() << "Stopping Realsense";
    pipe.stop();
}


QImage RealsenseWorker::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    if (size) *size = QSize(m_width, m_height);

    return image;
}
