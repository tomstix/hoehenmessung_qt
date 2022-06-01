#include "realsense.h"
#include <QDebug>

void Realsense::start()
{
    if (!started)
    {
        qDebug() << "Starting Realsense with " << width << "x" << height;
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16);
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8);
        pipe_profile = pipe.start(cfg);
        auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        auto depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto intrinsics = depth_stream.get_intrinsics();
        std::vector<std::vector<double>> mat{{intrinsics.fx, 0, intrinsics.ppx},
                                             {0, intrinsics.fy, intrinsics.ppy},
                                             {0, 0, 1}};
        intrinsic_matrix = mat;
        started = true;
    }
}

void Realsense::stop()
{
    if (started)
    {
        qDebug() << "Stopping Realsense";
        started = false;
        pipe.stop();
    }
}

void Realsense::setResolution(Resolution res_)
{
    res = res_;
    switch (res)
    {
    case RES_424_240:
        width = 424;
        height = 240;
        break;
    case RES_480_270:
        width = 480;
        height = 270;
        break;
    case RES_640_360:
        width = 640;
        height = 360;
        break;
    case RES_640_480:
        width = 640;
        height = 480;
        break;
    case RES_1280_720:
        width = 1280;
        height = 720;
        break;
    case RES_1280_800:
        width = 1280;
        height = 800;
        break;
    default:
        break;
    }
    qDebug() << "Resolution set to " << width << "x" << height;
    emit resolutionChanged();
}

Realsense::Resolution Realsense::resolution() const
{
    return res;
}