#pragma once

#include <QObject>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QQuickImageProvider>
#include <QDebug>
#include <QThread>

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <Eigen/Core>

struct PointcloudOptions
{
    Q_GADGET
public:
    bool enable_processing = true;
    bool filter_x = true;
    float x_min = -5.5F / 2.0F;
    float x_max = 5.5F / 2.0F;
    bool filter_y = true;
    float y_min = 0.0;
    float y_max = 5.5F;
    bool filter_z = true;
    float z_min = 0.2F;
    float z_max = 15.0F;
    float voxel_size = 0.01F;
    float ransac_threshold = 0.05F;
    float ransac_angle_max = 20.0F;
    int ransac_iterations = 1000;
    float ma_alpha = 0.1F;
    Q_PROPERTY(bool enable_processing MEMBER enable_processing)
    Q_PROPERTY(bool filter_x MEMBER filter_x)
    Q_PROPERTY(float x_min MEMBER x_min)
    Q_PROPERTY(float x_max MEMBER x_max)
    Q_PROPERTY(bool filter_y MEMBER filter_y)
    Q_PROPERTY(float y_min MEMBER y_min)
    Q_PROPERTY(float y_max MEMBER y_max)
    Q_PROPERTY(bool filter_z MEMBER filter_z)
    Q_PROPERTY(float z_min MEMBER z_min)
    Q_PROPERTY(float z_max MEMBER z_max)
    Q_PROPERTY(float voxel_size MEMBER voxel_size)
    Q_PROPERTY(float ransac_threshold MEMBER ransac_threshold)
    Q_PROPERTY(float ransac_angle_max MEMBER ransac_angle_max)
    Q_PROPERTY(int ransac_iterations MEMBER ransac_iterations)
    Q_PROPERTY(float ma_alpha MEMBER ma_alpha)
};
Q_DECLARE_METATYPE(PointcloudOptions)

class RealsenseWorker : public QThread, public QQuickImageProvider
{
    Q_OBJECT
    Q_PROPERTY(Resolution resolution READ resolution WRITE setResolution NOTIFY resolutionChanged)
    Q_PROPERTY(int width READ width NOTIFY widthChanged)
    Q_PROPERTY(int height READ height NOTIFY heightChanged)
    Q_PROPERTY(bool running READ running NOTIFY isRunningChanged)
    Q_PROPERTY(PointcloudOptions pointcloudoptions READ getPointcloudoptions WRITE setPointcloudoptions NOTIFY pointcloudoptionsChanged)
    Q_PROPERTY(float distanceRaw READ distanceRaw NOTIFY newFrameReady)
    Q_PROPERTY(int frameTime READ frameTime NOTIFY frameTimeChanged)
    Q_PROPERTY(bool tared MEMBER tared NOTIFY tareChanged)

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

    PointcloudOptions getPointcloudoptions() const
    {
        return pointcloudoptions;
    }
    void setPointcloudoptions(PointcloudOptions po)
    {
        pointcloudoptions = po;
        emit pointcloudoptionsChanged();
    }

    int width() const;
    int height() const;

    bool running() const;

    float distanceRaw() const;

    int frameTime() const;

public slots:
    void stop();
    void tare();
    void resetTare();

signals:
    void heightChanged(const int h);
    void widthChanged(const int w);
    void resolutionChanged();
    void isRunningChanged();
    void newFrameReady();
    void statusStringChanged();
    void pointcloudoptionsChanged();
    void frameTimeChanged();
    void tareChanged();
    void sendCANHeight(int id, QByteArray data, bool extended);

protected:
    void run() override;

private:
    rs2::config cfg;
    std::shared_ptr<rs2::pipeline> pipe = std::make_shared<rs2::pipeline>();
    rs2::pipeline_profile pipe_profile;
    Resolution res = RES_640_480;
    int m_width;
    int m_height;
    std::vector<std::vector<double>> intrinsic_matrix;
    bool m_isRunning = false;
    bool m_abortFlag = false;

    std::shared_ptr<Eigen::Affine3f> transform_mat = std::make_shared<Eigen::Affine3f>(Eigen::Affine3f::Identity());
    bool tared = false;

    QImage *colorImage = new QImage(1280, 720, QImage::Format_RGB888);
    std::chrono::_V2::system_clock::time_point lastFrameTimestamp = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds frameTime_ms = std::chrono::milliseconds::zero();

    pcl::PointCloud<pcl::PointXYZ>::Ptr rsDepthFrameToPCLCloud(std::unique_ptr<rs2::depth_frame> depth_frame) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr) const;
    void projectPointsToImage(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, QImage *image) const;

    std::shared_ptr<Eigen::Vector4f> groundPlaneCoefficients = std::make_shared<Eigen::Vector4f>(0, 0, 0, 0);
    std::shared_ptr<rs2_intrinsics> intrinsics = std::make_shared<rs2_intrinsics>();
    PointcloudOptions pointcloudoptions;
};
