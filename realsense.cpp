#include "realsense.h"

#include <QFile>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
int RealsenseWorker::frameTime() const
{
    return (int)frameTime_ms.count();
}
void RealsenseWorker::stop()
{
    m_abortFlag = true;
}
void RealsenseWorker::loadExtrinsics()
{
    using json = nlohmann::json;
    using namespace Eigen;

    QFile extrinsicsJsonFile("extrinsics.json");
    if (!extrinsicsJsonFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "Could not open file!";
        return;
    }

    QTextStream in(&extrinsicsJsonFile);
    auto j = json::parse(in.readAll().toStdString());

    auto r = j["rotate"].get<std::vector<float>>();
    auto t = j["translate"].get<std::vector<float>>();

    Matrix3f rot_matrix(r.data());

    transform_mat->rotate(rot_matrix);
    transform_mat->translation() << t.at(0), t.at(1), t.at(2);

    tared = true;
    emit tareChanged();
}
void RealsenseWorker::tare()
{
    using namespace std;
    using namespace Eigen;
    using json = nlohmann::json;

    Vector3f plane_vec = groundPlaneCoefficients->head<3>();
    Vector3f y_vector = {0.0, -1.0, 0.0};
    Vector3f rot_vector = y_vector.cross(plane_vec);
    rot_vector.normalize();
    float dist = groundPlaneCoefficients->w();
    float angle_cos = y_vector.dot(plane_vec);
    float angle = acos(angle_cos);

    Matrix3f rot_matrix;
    rot_matrix = AngleAxis(-angle, rot_vector);

    transform_mat->rotate(rot_matrix);
    transform_mat->translation() << 0.0, -dist, 0.0;

    pointcloudoptions.y_min = -2.0F;
    pointcloudoptions.y_max = 2.0F;
    groundPlaneCoefficients->w() = 0.0F;

    std::vector<float> rotateVec(rot_matrix.data(), rot_matrix.data() + rot_matrix.size());

    std::vector<float> translateVec(transform_mat->translation().data(), transform_mat->translation().data() + transform_mat->translation().size());

    json transformJson;
    transformJson["name"] = "extrinsics";
    transformJson["rotate"] = rotateVec;
    transformJson["translate"] = translateVec;

    QFile extrinsicsJsonFile("extrinsics.json");
    if (!extrinsicsJsonFile.open(QIODevice::WriteOnly))
    {
        qDebug() << "Could not open file!";
        return;
    }

    QTextStream out(&extrinsicsJsonFile);
    std::stringstream ss;
    ss << transformJson;
    out << QString::fromStdString(ss.str());
    tared = true;
    emit tareChanged();
}

void RealsenseWorker::resetTare()
{
    *transform_mat = Eigen::Affine3f::Identity().inverse();
    pointcloudoptions.y_min = 0.0F;
    pointcloudoptions.y_max = 5.5F;
    tared = false;
    emit tareChanged();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealsenseWorker::rsDepthFrameToPCLCloud(std::unique_ptr<rs2::depth_frame> depth_frame) const
{
    rs2::pointcloud pc;
    rs2::points points_cam = pc.calculate(*depth_frame);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points_cam.get_profile().as<rs2::video_stream_profile>();
    pclCloud->width = sp.width();
    pclCloud->height = sp.height();
    pclCloud->is_dense = false;
    pclCloud->points.resize(points_cam.size());
    const auto cloud_vertices_ptr = points_cam.get_vertices();
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
    if (tared)
    {
        pcl::transformPointCloud(*pclCloud, *pclCloud, transform_mat->inverse());
    }
    QPainter painter(image);
    QPen pen = painter.pen();
    pen.setColor(Qt::red);
    pen.setWidthF(pointcloudoptions.voxel_size * 100.0F);
    painter.setPen(pen);
#pragma omp parallel for default(none) shared(image, pclCloud)
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> cropBox;
    pcl::VoxelGrid<pcl::PointXYZ> downsample;

    // downsample PointCloud
    downsample.setInputCloud(pclCloud);
    downsample.setLeafSize(pointcloudoptions.voxel_size, pointcloudoptions.voxel_size, pointcloudoptions.voxel_size);
    downsample.filter(*pclCloud);

    // crop PointCloud
    Eigen::Vector4f min = {pointcloudoptions.x_min, pointcloudoptions.y_min, pointcloudoptions.z_min, 1.0F};
    Eigen::Vector4f max = {pointcloudoptions.x_max, pointcloudoptions.y_max, pointcloudoptions.z_max, 1.0F};

    // transform point cloud to world coordinates
    if (tared)
    {
        pcl::transformPointCloud(*pclCloud, *pclCloud, *transform_mat);
    }
    
    cropBox.setInputCloud(pclCloud);
    cropBox.setMin(min);
    cropBox.setMax(max);
    cropBox.filter(*pclCloud);

    // detect ground plane using ransac and perpendicular plane model
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr groundPlaneModel(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(pclCloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> groundPlaneRansac(groundPlaneModel);
    std::vector<int> groundPlaneInliers;
    groundPlaneModel->setAxis(Eigen::Vector3f(0.0, -1.0, 0.0));
    groundPlaneModel->setEpsAngle(pointcloudoptions.ransac_angle_max * M_PI / 180.0);
    groundPlaneRansac.setDistanceThreshold(pointcloudoptions.ransac_threshold);
    groundPlaneRansac.setMaxIterations(pointcloudoptions.ransac_iterations);
    groundPlaneRansac.setNumberOfThreads(0);
    if (bool success = groundPlaneRansac.computeModel())
    {
        groundPlaneRansac.getInliers(groundPlaneInliers);
        pcl::copyPointCloud(*pclCloud, groundPlaneInliers, *groundPlaneCloud);

        Eigen::VectorXf groundPlaneCoefficientsRaw;
        groundPlaneRansac.getModelCoefficients(groundPlaneCoefficientsRaw);

        // flip plane if upside down
        if (groundPlaneCoefficientsRaw.w() < 0)
        {
            groundPlaneCoefficientsRaw = -groundPlaneCoefficientsRaw;
        }

        // moving average of plane equation
        *groundPlaneCoefficients = (Eigen::Vector4f)(groundPlaneCoefficientsRaw.head<4>() * pointcloudoptions.ma_alpha + *groundPlaneCoefficients * (1.0F - pointcloudoptions.ma_alpha));
    }

    return groundPlaneCloud;
}

void RealsenseWorker::startStreaming()
{
    qDebug() << "Starting Realsense with " << m_width << "x" << m_height;
    cfg.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_RGB8, 30);
    pipe_profile = pipe->start(cfg);
    auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    auto depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    *intrinsics = depth_stream.get_intrinsics();

    m_isRunning = true;
    m_abortFlag = false;
    emit isRunningChanged();
}

void RealsenseWorker::stopStreaming()
{
    pipe->stop();
    qDebug() << "Realsense stopped";
    m_isRunning = false;
    emit isRunningChanged();
}

std::shared_ptr<rs2::frameset> RealsenseWorker::wait_for_frames(unsigned int timeout) const
{
    auto frames = pipe->wait_for_frames(timeout);
    return std::make_shared<rs2::frameset>(frames);
}

void RealsenseWorker::run()
try
{
    startStreaming();

    while (!m_abortFlag)
    {
        auto frames = wait_for_frames();

        if (*frames)
        {
            auto color_frame = frames->get_color_frame();
            auto depth_frame = frames->get_depth_frame();

            auto vf = color_frame.as<rs2::video_frame>();
            if (color_frame.get_profile().format() == RS2_FORMAT_RGB8)
            {
                *colorImage = QImage((uchar *)color_frame.get_data(), m_width, m_height, m_width * 3, QImage::Format_RGB888);
            }
            else
            {
                qDebug() << "Wrong format for color frame!";
            }
            if (depth_frame.get_profile().format() == RS2_FORMAT_Z16)
            {
                *depthImage = QImage((uchar *)depth_frame.get_data(), m_width, m_height, m_width * 2, QImage::Format_Grayscale16);
            }
            else
            {
                qDebug() << "Wrong format for depth frame!";
            }
            auto pclCloud = rsDepthFrameToPCLCloud(std::make_unique<rs2::depth_frame>(depth_frame));
            auto groundPlaneCloud = processPointcloud(pclCloud);
            if (paintPoints)
            {
                projectPointsToImage(groundPlaneCloud, colorImage);
            }

            emit newFrameReady();
            QByteArray heightData;
            heightData.append((char)(int16_t)(groundPlaneCoefficients->w() * 100.0));
            heightData.append((int16_t)(groundPlaneCoefficients->w() * 100.0) >> 8);
            emit sendCANHeight(100, heightData, true);

            auto time_now = std::chrono::high_resolution_clock::now();
            frameTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - lastFrameTimestamp);
            emit frameTimeChanged();
            lastFrameTimestamp = std::chrono::high_resolution_clock::now();
        }
    }
    stopStreaming();
}
catch (const rs2::error &e)
{
    std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
}

QImage RealsenseWorker::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    QString id_ = id;
    id_.chop(2);

    if (size)
        *size = QSize(m_width, m_height);

    if (id_ == "color")
        return *colorImage;
    else if (id_ == "depth")
        return *depthImage;

    return QPixmap(requestedSize.width(), requestedSize.height()).toImage();
}
