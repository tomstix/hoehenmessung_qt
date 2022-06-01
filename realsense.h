#ifndef REALSENSE_H
#define REALSENSE_H

#include <QObject>
#include <QImage>

#include <librealsense2/rs.hpp>

class Realsense : public QObject
{
    Q_OBJECT
    Q_PROPERTY(Resolution resolution READ resolution WRITE setResolution NOTIFY resolutionChanged)
public:
    enum Resolution
    {
        RES_424_240,
        RES_480_270,
        RES_640_360,
        RES_640_480,
        RES_1280_720,
        RES_1280_800
    };
    Q_ENUM(Resolution)
    void setResolution(Resolution res_);
    Resolution resolution() const;

signals:
    void resolutionChanged();
    

public slots:
    void start();
    void stop();

private:
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pipeline_profile pipe_profile;
    rs2::frameset frames;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::frame depth_frame;
    rs2::frame color_frame;
    Resolution res = RES_1280_720;
    int width;
    int height;
    std::vector<std::vector<double>> intrinsic_matrix;
    bool started = false;
};

#endif // REALSENSE_H
