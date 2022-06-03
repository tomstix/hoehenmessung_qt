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
bool RealsenseWorker::running() const
{
    return m_isRunning;
}
float RealsenseWorker::distanceRaw() const
{
    return groundPlaneCoefficients->w();
}
void RealsenseWorker::stop()
{
    m_isRunning = false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealsenseWorker::rsDepthFrameToPCLCloud(std::unique_ptr<rs2::depth_frame> depth_frame) const
{
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(*depth_frame);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pclCloud->width = sp.width();
    pclCloud->height = sp.height();
    pclCloud->is_dense = false;
    pclCloud->points.resize(points.size());
    const auto cloud_vertices_ptr = points.get_vertices();
#pragma omp parallel for default(none) shared(pclCloud, cloud_vertices_ptr)
    for (std::size_t index = 0; index < pclCloud->size(); index++)
    {
        const auto ptr = cloud_vertices_ptr + index;
        auto &p = (*pclCloud)[index];

        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
    }

    return pclCloud;
}

void RealsenseWorker::projectPointsToImage(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, QImage *image) const
{
    QPainter painter(image);
    painter.setPen(Qt::green);
    for (auto point : pclCloud->points)
    {
        float pix[2];
        float p[] = {point.x, point.y, point.z};
        const rs2_intrinsics intrin = *intrinsics;
        rs2_project_point_to_pixel(pix, &intrin, p);
        painter.drawPoint((int)std::round(pix[0]), (int)std::round(pix[1]));
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealsenseWorker::processPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> cropBox;
    pcl::VoxelGrid<pcl::PointXYZ> downsample;

    // crop PointCloud
    Eigen::Vector4f min = {pointcloudoptions.x_min, pointcloudoptions.y_min, pointcloudoptions.z_min, 1.0F};
    Eigen::Vector4f max = {pointcloudoptions.x_max, pointcloudoptions.y_max, pointcloudoptions.z_max, 1.0F};
    cropBox.setInputCloud(pclCloud);
    cropBox.setMin(min);
    cropBox.setMax(max);
    cropBox.filter(*cloud_cropped);

    // downsample PointCloud
    downsample.setInputCloud(cloud_cropped);
    downsample.setLeafSize(pointcloudoptions.voxel_size, pointcloudoptions.voxel_size, pointcloudoptions.voxel_size);
    downsample.filter(*cloud_downsampled);

    // detect ground plane using ransac and perpendicular plane model
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr groundPlaneModel(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(cloud_downsampled));
    pcl::RandomSampleConsensus<pcl::PointXYZ> groundPlaneRansac(groundPlaneModel);
    std::vector<int> groundPlaneInliers;
    groundPlaneModel->setAxis(Eigen::Vector3f(0.0,-1.0,0.0));
    groundPlaneModel->setEpsAngle(pointcloudoptions.ransac_angle_max * M_PI / 180.0);
    groundPlaneRansac.setDistanceThreshold(pointcloudoptions.ransac_threshold);
    groundPlaneRansac.setMaxIterations(pointcloudoptions.ransac_iterations);
    groundPlaneRansac.setNumberOfThreads(0);
    if (bool success = groundPlaneRansac.computeModel())
    {
        groundPlaneRansac.getInliers(groundPlaneInliers);
        pcl::copyPointCloud(*cloud_downsampled, groundPlaneInliers, *groundPlaneCloud);

        Eigen::VectorXf groundPlaneCoefficientsRaw;
        groundPlaneRansac.getModelCoefficients(groundPlaneCoefficientsRaw);
        if (groundPlaneCoefficientsRaw.w() < 0)
        {
            groundPlaneCoefficientsRaw = -groundPlaneCoefficientsRaw;
        }

        // moving average of plane equation
        *groundPlaneCoefficients = (Eigen::Vector4f)(groundPlaneCoefficientsRaw.head<4>() * pointcloudoptions.ma_alpha + *groundPlaneCoefficients*(1.0F-pointcloudoptions.ma_alpha));


    }

    return groundPlaneCloud;
}

void RealsenseWorker::run()
{
    qDebug() << "Starting Realsense with " << m_width << "x" << m_height;
    m_isRunning = true;
    emit isRunningChanged();
    cfg.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_RGB8, 30);
    pipe_profile = pipe.start(cfg);
    auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    auto depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    *intrinsics = depth_stream.get_intrinsics();

    while (m_isRunning)
    {
        frames = pipe.wait_for_frames();
        
        auto color_frame = frames.get_color_frame();
        auto depth_frame = frames.get_depth_frame();

        auto vf = color_frame.as<rs2::video_frame>();
        if (color_frame.get_profile().format() == RS2_FORMAT_RGB8)
        {
            *colorImage = QImage((uchar *)color_frame.get_data(), m_width, m_height, m_width * 3, QImage::Format_RGB888);
        }
        auto pclCloud = rsDepthFrameToPCLCloud(std::make_unique<rs2::depth_frame>(depth_frame));
        auto gpc = processPointcloud(pclCloud);
        projectPointsToImage(gpc, colorImage);

        emit newFrameReady();
    }
    qDebug() << "Stopping Realsense";
    emit isRunningChanged();
    pipe.stop();
}

QImage RealsenseWorker::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    if (size) *size = QSize(m_width, m_height);

    return *colorImage;
}
