#pragma once

#include <QObject>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QQuickImageProvider>
#include <QDebug>
#include <QThread>
#include <QFile>

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nlohmann/json.hpp>

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
    Q_PROPERTY(int width READ width NOTIFY resolutionChanged)
    Q_PROPERTY(int height READ height NOTIFY resolutionChanged)
    Q_PROPERTY(bool running READ running NOTIFY isRunningChanged)
    Q_PROPERTY(PointcloudOptions pointcloudoptions READ pointcloudoptions WRITE setPointcloudoptions NOTIFY pointcloudoptionsChanged)
    Q_PROPERTY(float distanceRaw READ distanceRaw NOTIFY newFrameReady)
    Q_PROPERTY(QPointF heightPoint READ heightPoint NOTIFY newHeightPoint)
    Q_PROPERTY(int frameTime READ frameTime NOTIFY frameTimeChanged)
    Q_PROPERTY(bool tared MEMBER m_tared NOTIFY tareChanged)
    Q_PROPERTY(bool paintPoints MEMBER m_paintPoints NOTIFY paintPointsChanged)
    Q_PROPERTY(QUrl bagFile READ bagFile WRITE setBagFile NOTIFY bagFileChanged)
    Q_PROPERTY(bool useBag MEMBER m_useBag NOTIFY useBagChanged)

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

    PointcloudOptions pointcloudoptions() const
    {
        return m_pointcloudoptions;
    }
    void setPointcloudoptions(PointcloudOptions const& po)
    {
        m_pointcloudoptions = po;
        emit pointcloudoptionsChanged();
    }

    int width() const;
    int height() const;
    bool running() const;
    float distanceRaw() const;
    int frameTime() const;
    QPointF heightPoint() const;

    QUrl bagFile() const;
    void setBagFile(QUrl url);

public slots:
    void stop();
    void tare();
    void resetTare();
    void loadExtrinsics();

signals:
    void resolutionChanged();
    void isRunningChanged();
    void newFrameReady();
    void pointcloudoptionsChanged();
    void frameTimeChanged();
    void tareChanged(bool isTared);
    void paintPointsChanged();
    void newHeightPoint();
    void bagFileChanged();
    void useBagChanged();
    void sendCANHeight(int id, QByteArray data, bool extended);

protected:
    void run() override;
    void startStreaming();
    void stopStreaming();
    pcl::PointCloud<pcl::PointXYZ>::Ptr rsDepthFrameToPCLCloud(std::unique_ptr<rs2::depth_frame> depth_frame) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr) const;
    void projectPointsToPixmap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud) const;

private:
    std::shared_ptr<rs2::frameset> get_frames(unsigned int timeout = 15000U) const;

    QUrl m_bagFile;
    bool m_useBag = false;
    std::shared_ptr<rs2::pipeline> pipe = std::make_shared<rs2::pipeline>();
    rs2::pipeline_profile pipe_profile;
    Resolution m_resolution = RES_640_480;
    int m_width;
    int m_height;
    std::vector<std::vector<double>> intrinsic_matrix;
    bool m_isRunning = false;
    bool m_abortFlag = false;

    bool m_paintPoints = true;

    std::shared_ptr<Eigen::Affine3f> transform_mat = std::make_shared<Eigen::Affine3f>(Eigen::Affine3f::Identity());
    std::shared_ptr<Eigen::Affine3f> transform_mat_inv = std::make_shared<Eigen::Affine3f>(Eigen::Affine3f::Identity());
    bool m_tared = false;

    std::shared_ptr<QImage> colorImage = std::make_shared<QImage>(640, 480, QImage::Format_RGB888);
    std::shared_ptr<QImage> depthImage = std::make_shared<QImage>(640, 480, QImage::Format_Grayscale16);
    std::shared_ptr<QPixmap> planePixmap = std::make_shared<QPixmap>();

    std::chrono::_V2::system_clock::time_point m_lastFrameTimestamp = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds m_frameTime_ms = std::chrono::milliseconds::zero();

    QPointF m_heightPoint;

    std::shared_ptr<Eigen::Vector4f> groundPlaneCoefficients = std::make_shared<Eigen::Vector4f>(0, 0, 0, 0);
    std::shared_ptr<rs2_intrinsics> intrinsics = std::make_shared<rs2_intrinsics>();
    PointcloudOptions m_pointcloudoptions;
};
