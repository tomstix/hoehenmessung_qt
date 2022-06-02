#ifndef REALSENSE_H
#define REALSENSE_H

#include <QObject>
#include <QImage>
#include <QQuickImageProvider>
#include <QDebug>
#include <QThread>

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

class RealsenseWorker : public QThread, public QQuickImageProvider
{
    Q_OBJECT
    Q_PROPERTY(Resolution resolution READ resolution WRITE setResolution NOTIFY resolutionChanged)
    Q_PROPERTY(int width READ width NOTIFY widthChanged)
    Q_PROPERTY(int height READ height NOTIFY heightChanged)
    Q_PROPERTY(bool running READ running NOTIFY isRunningChanged)

public:
    RealsenseWorker() : QQuickImageProvider(QQuickImageProvider::Image)
    {
    }

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

    enum Resolution
    {
        RES_424_240,
        RES_480_270,
        RES_640_360,
        RES_640_480,
        RES_1280_720
    };
    Q_ENUM(Resolution)
    void setResolution(Resolution res_);
    Resolution resolution() const;

    int width() const;
    int height() const;

    bool running() const;

public slots:
    void stop();

signals:
    void heightChanged(const int h);
    void widthChanged(const int w);
    void resolutionChanged();
    void isRunningChanged();
    void colorImageReady();

protected:
    void run() override;

private:
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pipeline_profile pipe_profile;
    rs2::frameset frames;
    Resolution res = RES_1280_720;
    int m_width;
    int m_height;
    std::vector<std::vector<double>> intrinsic_matrix;
    bool m_isRunning;
    QImage colorImage = QPixmap(1280, 720).toImage();

    pcl::PointCloud<pcl::PointXYZ>::Ptr processDepthFrame(std::unique_ptr<rs2::depth_frame> depth_frame) const;
    struct PointcloudOptions
    {
        bool enable_processing = true;
        bool filter_x = true;
        float x_min = -5.5F/2.0F;
        float x_max = 5.5F/2.0F;
        bool filter_y = true;
        float y_min = 0.5F;
        float y_max = 4.5F;
        bool filter_z = true;
        float z_min = 0.2F;
        float z_max = 20.0F;
        float voxel_size = 0.01F;
        float ransac_threshold = 0.05F;
        float ransac_angle_max = 20.0F;
        int ransac_iterations = 1000;
        float ma_alpha = 0.1F;
    }; PointcloudOptions pointcloudoptions;
};

#endif // REALSENSE_H
