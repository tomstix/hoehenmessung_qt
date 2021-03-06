#include "realsense.h"

#include <librealsense2/rsutil.h>

Point3D::Point3D(float newX, float newY, float newZ) : x(newX), y(newY), z(newZ) {}

Point3DList::Point3DList(QObject *parent) : QAbstractListModel(parent) {}

void Point3DList::resetPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    auto newPoints = std::make_shared<QVector<Point3D>>();
#pragma omp parallel for default(none) shared(cloud, newPoints)
    for (auto p : cloud->points)
    {
        if (p.x == p.x && p.y == p.y && p.z == p.z && p.y != 0x7fd980)
        {
            *newPoints << Point3D(p.x, -p.y, p.z);
        }
    }
    beginResetModel();
    m_points = newPoints;
    endResetModel();
}

int Point3DList::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return m_points->count();
}

QVariant Point3DList::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() >= m_points->count())
        return QVariant();
    const auto& points = *m_points;
    const Point3D point = points[index.row()];
    if (role == xRole)
        return point.x;
    if (role == yRole)
        return point.y;
    if (role == zRole)
        return point.z;
    return QVariant();
}

QHash<int, QByteArray> Point3DList::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[xRole] = "x";
    roles[yRole] = "y";
    roles[zRole] = "z";
    return roles;
}

Point3DList *RealsenseWorker::groundPlanePoints3D() const
{
    return m_groundPlanePointsModel;
}

Point3DList *RealsenseWorker::restPoints3D() const
{
    return m_restPointsModel;
}

void RealsenseWorker::setResolution(Resolution res_)
{
    m_resolution = res_;
    switch (m_resolution)
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
    emit resolutionChanged();
}
bool RealsenseWorker::paused() const
{
    return m_paused;
}

void RealsenseWorker::setPaused(bool newPaused)
{
    if (m_paused == newPaused)
        return;
    m_paused = newPaused;

    if (auto pb = m_device->as<rs2::playback>())
    {
        if (m_paused) pb.pause();
        else pb.resume();
    }

    emit pausedChanged();
}
RealsenseWorker::Resolution RealsenseWorker::resolution() const
{
    return m_resolution;
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
    return m_groundPlaneCoefficients->w();
}
QPointF RealsenseWorker::heightPoint() const
{
    return m_heightPoint;
}
QUrl RealsenseWorker::bagFile() const
{
    return m_bagFile;
}
void RealsenseWorker::setBagFile(QUrl url)
{
    m_bagFile = url;
    qDebug() << "BAG file set to " << m_bagFile;
    emit bagFileChanged();
}
QUrl RealsenseWorker::recordFile() const
{
    return m_recordFile;
}
void RealsenseWorker::setRecordFile(QUrl url)
{
    m_recordFile = url;
    qDebug() << "BAG file set to " << m_recordFile;
    emit recordFileChanged();
}

int RealsenseWorker::frameTime() const
{
    return (int)m_frameTime_ms.count();
}
void RealsenseWorker::stop()
{
    m_abortFlag = true;
}
void RealsenseWorker::loadExtrinsics()
{
    using json = nlohmann::json;
    using namespace Eigen;

    qDebug() << "Loading extrinsics";

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

    m_transform_mat->rotate(rot_matrix);
    m_transform_mat->translation() << t.at(0), t.at(1), t.at(2);
    m_transform_mat_inv = std::make_shared<Affine3f>(m_transform_mat->inverse());

    m_tared = true;
    emit tareChanged(m_tared);
}
void RealsenseWorker::tare()
{
    using namespace std;
    using namespace Eigen;
    using json = nlohmann::json;

    Vector3f plane_vec = m_groundPlaneCoefficients->head<3>();
    Vector3f y_vector = {0.0, 1.0, 0.0};
    Vector3f rot_vector = y_vector.cross(plane_vec);
    rot_vector.normalize();
    float dist = -m_groundPlaneCoefficients->w();
    float angle_cos = y_vector.dot(plane_vec);
    float angle = acos(angle_cos);

    Matrix3f rot_matrix;
    rot_matrix = AngleAxis(-angle, rot_vector);

    m_transform_mat->rotate(rot_matrix);
    m_transform_mat->translation() << 0.0, -dist, 0.0;
    m_transform_mat_inv = std::make_shared<Affine3f>(m_transform_mat->inverse());

    m_groundPlaneCoefficients->w() = 0.0F;

    std::vector<float> rotateVec(rot_matrix.data(), rot_matrix.data() + rot_matrix.size());

    std::vector<float> translateVec(m_transform_mat->translation().data(), m_transform_mat->translation().data() + m_transform_mat->translation().size());

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
    m_tared = true;
    emit tareChanged(m_tared);
}

void RealsenseWorker::resetTare()
{
    *m_transform_mat = Eigen::Affine3f::Identity().inverse();
    m_tared = false;
    emit tareChanged(m_tared);
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

std::unique_ptr<QPixmap> RealsenseWorker::projectPointsToPixmap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud) const
{
    if (m_tared)
    {
        pcl::transformPointCloud(*pclCloud, *pclCloud, *m_transform_mat_inv);
    }
    auto pixmap = std::make_unique<QPixmap>(m_width, m_height);
    pixmap->fill(Qt::transparent);
    QPainter painter(pixmap.get());
    QPen pen = painter.pen();
    pen.setColor(Qt::red);
    pen.setWidthF(m_pointcloudoptions.voxel_size * 100.0F);
    painter.setPen(pen);
#pragma omp parallel for default(none) shared(image, pclCloud)
    for (auto point : pclCloud->points)
    {
        float pix[2];
        float p[] = {point.x, point.y, point.z};
        const rs2_intrinsics intrin = *m_intrinsics;
        rs2_project_point_to_pixel(pix, &intrin, p);
        painter.drawPoint((int)std::round(pix[0]), (int)std::round(pix[1]));
    }
    return pixmap;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealsenseWorker::processPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr restPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> cropBox;
    pcl::VoxelGrid<pcl::PointXYZ> downsample;

    // crop PointCloud
    Eigen::Vector4f min = {m_pointcloudoptions.x_min, m_pointcloudoptions.y_min, m_pointcloudoptions.z_min, 1.0F};
    Eigen::Vector4f max = {m_pointcloudoptions.x_max, m_pointcloudoptions.y_max, m_pointcloudoptions.z_max, 1.0F};
    min = m_transform_mat->matrix() * min;
    max = m_transform_mat->matrix() * max;
    // reset x and y values
    min(0) = m_pointcloudoptions.x_min;
    max(0) = m_pointcloudoptions.x_max;
    min(1) = m_pointcloudoptions.y_min;
    max(1) = m_pointcloudoptions.y_max;
    cropBox.setInputCloud(pclCloud);
    cropBox.setMin(min);
    cropBox.setMax(max);
    cropBox.filter(*pclCloud);

    // downsample PointCloud
    downsample.setInputCloud(pclCloud);
    downsample.setLeafSize(m_pointcloudoptions.voxel_size, m_pointcloudoptions.voxel_size, m_pointcloudoptions.voxel_size);
    downsample.filter(*pclCloud);

    // transform point cloud to vehicle coordinates
    if (m_tared)
    {
        pcl::transformPointCloud(*pclCloud, *pclCloud, *m_transform_mat);
    }

    // detect ground plane using ransac and perpendicular plane model
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr groundPlaneModel(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(pclCloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> groundPlaneRansac(groundPlaneModel);
    std::vector<int> groundPlaneInliers;
    groundPlaneModel->setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
    groundPlaneModel->setEpsAngle(m_pointcloudoptions.ransac_angle_max * M_PI / 180.0);
    groundPlaneRansac.setDistanceThreshold(m_pointcloudoptions.ransac_threshold);
    groundPlaneRansac.setMaxIterations(m_pointcloudoptions.ransac_iterations);
    groundPlaneRansac.setNumberOfThreads(0);
    if (bool success = groundPlaneRansac.computeModel())
    {
        groundPlaneRansac.getInliers(groundPlaneInliers);
        pcl::copyPointCloud(*pclCloud, groundPlaneInliers, *groundPlaneCloud);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers->indices = groundPlaneInliers;
        extract.setInputCloud(pclCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*restPointCloud);
        // copy Points to Point3DModel List
        m_groundPlanePointsModel->resetPoints(groundPlaneCloud);
        m_restPointsModel->resetPoints(restPointCloud);

        Eigen::VectorXf groundPlaneCoefficientsRaw;
        groundPlaneRansac.getModelCoefficients(groundPlaneCoefficientsRaw);

        // flip plane if upside down
        if (groundPlaneCoefficientsRaw.y() < 0)
        {
            groundPlaneCoefficientsRaw = -groundPlaneCoefficientsRaw;
        }

        // moving average of plane equation
        *m_groundPlaneCoefficients = (Eigen::Vector4f)(groundPlaneCoefficientsRaw.head<4>() * m_pointcloudoptions.ma_alpha + *m_groundPlaneCoefficients * (1.0F - m_pointcloudoptions.ma_alpha));
    }

    return groundPlaneCloud;
}

void printOptions(rs2::sensor sensor, std::vector<rs2_option> options)
{
    for (auto option : options)
        try
        {
            qDebug() << sensor.get_option_name(option) << " : " << sensor.get_option_description(option);
        }
        catch (const rs2::error &e)
        {
            std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
        }
}

void RealsenseWorker::startStreaming()
{
    try
    {
        rs2::config cfg;
        if (m_useBag)
        {
            auto url = m_bagFile.toString(QUrl::RemoveScheme);
            qDebug() << "Starting Stream from BAG file: " << url;
            cfg.enable_device_from_file(url.toStdString());
            m_pipe_profile = m_pipe->start(cfg);
        }
        else
        {
            qDebug() << "Starting Realsense with " << m_width << "x" << m_height;
            cfg.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_YUYV, 30);
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, m_width, m_height, RS2_FORMAT_Y8, 30);

            if (m_record)
            {
                auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
                auto lt = std::put_time(&tm, "%Y%m%d_%H%M%S");
                std::stringstream ss;
                ss << m_recordFile.toString(QUrl::RemoveScheme).toStdString() << '/' << lt << ".bag";
                std::cout << "Recording to: " << ss.str() << std::endl;
                cfg.enable_record_to_file(ss.str());
            }

            m_pipe_profile = m_pipe->start(cfg);
            auto sensor = m_pipe_profile.get_device().first<rs2::depth_sensor>();
            qDebug() << "Connecting to " << m_pipe_profile.get_device().get_info(RS2_CAMERA_INFO_NAME);
            // sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        }

        m_device = std::make_shared<rs2::device>(m_pipe->get_active_profile().get_device());

        auto depth_stream = m_pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        *m_intrinsics = depth_stream.get_intrinsics();

        m_align_to_color = std::make_shared<rs2::align>(RS2_STREAM_COLOR);

        m_isRunning = true;
        m_abortFlag = false;
        emit isRunningChanged();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
        m_isRunning = false;
        m_abortFlag = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void RealsenseWorker::stopStreaming()
{
    m_pipe->stop();
    m_pipe = std::make_shared<rs2::pipeline>();
    qDebug() << "Realsense stopped";
    m_isRunning = false;
    m_colorImage = std::make_shared<QImage>(640, 480, QImage::Format_RGB888);
    m_depthImage = std::make_shared<QImage>(640, 480, QImage::Format_Grayscale16);
    m_infraredImage = std::make_shared<QImage>(640, 480, QImage::Format_RGB888);
    emit isRunningChanged();
}

uchar *convert_yuyv_to_rgb(const uchar *yuyv_image, int width, int height) // https://stackoverflow.com/questions/9098881/convert-from-yuv-to-rgb-in-c-android-ndk
{
    auto *rgb_image = new unsigned char[width * height * 3]; // width and height of the image to be converted

    int y;
    int cr;
    int cb;

    double r;
    double g;
    double b;

    for (int i = 0, j = 0; i < width * height * 3; i += 6, j += 4)
    {
        // first pixel
        y = yuyv_image[j];
        cb = yuyv_image[j + 1];
        cr = yuyv_image[j + 3];

        r = y + (1.4065 * (cr - 128));
        g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
        b = y + (1.7790 * (cb - 128));

        // This prevents colour distortions in your rgb image
        if (r < 0)
            r = 0;
        else if (r > 255)
            r = 255;
        if (g < 0)
            g = 0;
        else if (g > 255)
            g = 255;
        if (b < 0)
            b = 0;
        else if (b > 255)
            b = 255;

        rgb_image[i] = (unsigned char)r;
        rgb_image[i + 1] = (unsigned char)g;
        rgb_image[i + 2] = (unsigned char)b;

        // second pixel
        y = yuyv_image[j + 2];
        cb = yuyv_image[j + 1];
        cr = yuyv_image[j + 3];

        r = y + (1.4065 * (cr - 128));
        g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
        b = y + (1.7790 * (cb - 128));

        if (r < 0)
            r = 0;
        else if (r > 255)
            r = 255;
        if (g < 0)
            g = 0;
        else if (g > 255)
            g = 255;
        if (b < 0)
            b = 0;
        else if (b > 255)
            b = 255;

        rgb_image[i + 3] = (unsigned char)r;
        rgb_image[i + 4] = (unsigned char)g;
        rgb_image[i + 5] = (unsigned char)b;
    }
    return rgb_image;
}

void RealsenseWorker::run()
try
{
    loadExtrinsics();
    startStreaming();
    auto start_time = std::chrono::system_clock::now();

    while (!m_abortFlag)
    {
        if (m_device->as<rs2::playback>())
        {
            if (!m_paused)
            {
                rs2::frameset frameset;
                bool s = m_pipe->poll_for_frames(&frameset);
                if (!s)
                {
                    m_frameset.reset();
                }
                else m_frameset = std::make_shared<rs2::frameset>(frameset);
            }
        }
        else
        {
            m_frameset = std::make_shared<rs2::frameset>(m_pipe->wait_for_frames());
        }
        if (m_frameset)
        {
            *m_frameset = m_align_to_color->process(*m_frameset);
            auto color_frame = m_frameset->get_color_frame();
            auto depth_frame = m_frameset->get_depth_frame();
            auto ir_frame = m_frameset->get_infrared_frame(1);

            m_width = color_frame.get_width();
            m_height = color_frame.get_height();

            // make QImages from Color and Depth frames
            if (color_frame.get_profile().format() == RS2_FORMAT_RGB8)
            {
                *m_colorImage = QImage((uchar *)color_frame.get_data(), m_width, m_height, m_width * 3, QImage::Format_RGB888);
            }
            else if (color_frame.get_profile().format() == RS2_FORMAT_YUYV)
            {
                *m_colorImage = QImage(convert_yuyv_to_rgb((uchar *)color_frame.get_data(), m_width, m_height), m_width, m_height, m_width * 3, QImage::Format_RGB888);
            }
            else
            {
                qDebug() << "Wrong format for color frame!";
            }
            if (depth_frame.get_profile().format() == RS2_FORMAT_Z16)
            {
                *m_depthImage = QImage((uchar *)depth_frame.get_data(), m_width, m_height, m_width * 2, QImage::Format_Grayscale16);
            }
            else
            {
                qDebug() << "Wrong format for depth frame!";
            }
            if (ir_frame)
            {
                if (ir_frame.get_profile().format() == RS2_FORMAT_Y8)
                {
                    *m_infraredImage = QImage((uchar *)ir_frame.get_data(), m_width, m_height, m_width, QImage::Format_Grayscale8);
                }
            }

            // Point Cloud Processing
            if (m_processPoints)
            {
                auto pclCloud = rsDepthFrameToPCLCloud(std::make_unique<rs2::depth_frame>(depth_frame));
                m_groundPlaneCloud = processPointcloud(pclCloud);
            }
            emit newFrameReady();

            emit newHeight(m_groundPlaneCoefficients->w());

            // calculate frame time
            auto time_now = std::chrono::high_resolution_clock::now();
            m_frameTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - m_lastFrameTimestamp);
            emit frameTimeChanged();
            m_lastFrameTimestamp = time_now;

            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - start_time);
            long timestamp = now_ms.count();
            m_heightPoint.setX((qreal)timestamp);
            m_heightPoint.setY(m_groundPlaneCoefficients->w());
            emit newHeightPoint();
        }
        m_frameset.reset();
    }
    stopStreaming();
}
catch (const rs2::error &e)
{
    std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
    stopStreaming();
}

// QQuickImageProvider function
QImage RealsenseWorker::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    QString id_ = id;
    id_.chop(2);

    if (size)
        *size = QSize(m_width, m_height);

    auto im = m_colorImage;

    if (id_ == "depth")
        im = m_depthImage;
    else if (id_ == "infrared")
        im = m_infraredImage;

    if (m_processPoints && m_paintPoints)
    {
        auto pm = projectPointsToPixmap(m_groundPlaneCloud);
        QImage img(*im);
        QPainter p(&img);
        p.drawPixmap(0, 0, *pm);
        return img;
    }
    return *im;

    return QPixmap(requestedSize.width(), requestedSize.height()).toImage();
}
