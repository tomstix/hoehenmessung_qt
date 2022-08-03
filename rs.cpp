#include "rs.hpp"

#include <iomanip>
#include <string>

//
//
//  RealsenseDeviceList
//
//

RealsenseDeviceList::RealsenseDeviceList(QObject *parent) : QAbstractListModel(parent)
{
    try {
        auto devices = m_ctx.query_devices();
        for (const auto& dev : devices)
        {
            m_devices.append(dev);
        }
        auto devices_changed_callback = [&](rs2::event_information& info)
        {
            beginResetModel();
            m_devices.clear();
            auto devices = m_ctx.query_devices();
            endResetModel();
        };
        m_ctx.set_devices_changed_callback(devices_changed_callback);
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
    }
}

int RealsenseDeviceList::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent)
    return m_devices.size() + m_bagFile_url.isValid();
}

QVariant RealsenseDeviceList::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() > m_devices.size())
        return QVariant();
    if (index.row() == m_devices.size())
    {
        std::stringstream ss;
        ss << "File: " << m_bagFile_url.toDisplayString(QUrl::RemoveScheme).toStdString();
        return QString::fromStdString(ss.str());
    }
    auto device = m_devices[index.row()];
    auto info = rs2_camera_info(index.row());
    if (device.supports(info))
    {
        return device.get_info(info);
    }
    return QVariant();
}

QHash<int, QByteArray> RealsenseDeviceList::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[RS2_CAMERA_INFO_NAME_ROLE] = "NAME";
    roles[RS2_CAMERA_INFO_SERIAL_NUMBER_ROLE] = "SERIAL_NUMBER";
    roles[RS2_CAMERA_INFO_FIRMWARE_VERSION_ROLE] = "FIRMWARE_VERSION";
    roles[RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION_ROLE] = "RECOMMENDED_FIRMWARE_VERSION";
    roles[RS2_CAMERA_INFO_PHYSICAL_PORT_ROLE] = "PHYSICAL_PORT";
    roles[RS2_CAMERA_INFO_DEBUG_OP_CODE_ROLE] = "DEBUG_OP_CODE";
    roles[RS2_CAMERA_INFO_ADVANCED_MODE_ROLE] = "ADVANCED_MODE";
    roles[RS2_CAMERA_INFO_PRODUCT_ID_ROLE] = "PRODUCT_ID";
    roles[RS2_CAMERA_INFO_CAMERA_LOCKED_ROLE] = "CAMERA_LOCKED";
    roles[RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR_ROLE] = "USB_TYPE_DESCRIPTOR";
    roles[RS2_CAMERA_INFO_PRODUCT_LINE_ROLE] = "PRODUCT_LINE";
    roles[RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER_ROLE] = "ASIC_SERIAL_NUMBER";
    roles[RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID_ROLE] = "FIRWMARE_UPDATE_ID";
    roles[RS2_CAMERA_INFO_IP_ADDRESS_ROLE] = "IP_ADDRESS";
    roles[RS2_CAMERA_INFO_COUNT_ROLE] = "COUNT";
    return roles;
}

bool RealsenseDeviceList::use_bag() const
{
    return m_use_bag;
}

void RealsenseDeviceList::set_device(int index)
{
    if (index == m_devices.size() && m_bagFile_url.isValid())
    {
        m_selected_device = rs2::device();
        m_use_bag = true;
        emit use_bagChanged();
    }
    else if (m_devices.size() > 0)
    {
        m_selected_device = m_devices[index];
        m_use_bag = false;
        emit use_bagChanged();
        m_color_resolutions.clear();
        m_depth_resolutions.clear();
        for (auto& sensor : m_selected_device.query_sensors())
        {
            for (auto& profile : sensor.get_stream_profiles())
            {
                if (auto video = profile.as<rs2::video_stream_profile>())
                {
                    if (video.fps() == 30)
                    {
                        if ((strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME), "RGB Camera") == 0) && video.format() == RS2_FORMAT_RGB8)
                        {
                            std::stringstream ss;
                            ss << video.width() << "x" << video.height();
                            m_color_resolutions << QString::fromStdString(ss.str());
                            qDebug() << "Adding Color Resolution: " << QString::fromStdString(ss.str());
                        }
                        else if ((strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME), "Stereo Module") == 0) && video.format() == RS2_FORMAT_Z16)
                        {
                            std::stringstream ss;
                            ss << video.width() << "x" << video.height();
                            m_depth_resolutions << QString::fromStdString(ss.str());
                            qDebug() << "Adding Depth Resolution: " << QString::fromStdString(ss.str());
                        }
                    }
                }
            }
        }
        emit color_resolutionsChanged();
        emit depth_resolutionsChanged();
    }
}

const rs2::device &RealsenseDeviceList::selected_device() const
{
    return m_selected_device;
}

const QUrl &RealsenseDeviceList::bagFile_url() const
{
    return m_bagFile_url;
}

void RealsenseDeviceList::setBagFile_url(const QUrl &newBagFile_url)
{
    if (m_bagFile_url == newBagFile_url)
        return;
    beginInsertRows(QModelIndex(), m_devices.size(), m_devices.size());
    m_bagFile_url = newBagFile_url;
    endInsertRows();
    emit bagFile_urlChanged();
}

QStringList RealsenseDeviceList::color_resolutions() const
{
    return m_color_resolutions;
}

QStringList RealsenseDeviceList::depth_resolutions() const
{
    return m_depth_resolutions;
}

//
//
//  RealsenseDataProvider
//
//


RealsenseDataProvider::RealsenseDataProvider(QObject *parent) : QObject(parent)
{

}

//
//
//  RealsenseDataProviderLive
//
//


RealsenseDataProviderLive::RealsenseDataProviderLive(rs2::device device, int color_width, int color_height, int depth_width, int depth_height)
                                                : RealsenseDataProvider(), m_device(device)
{
    qDebug() << "Creating new RealsenseDataProviderLive";
    m_color_width = color_width;
    m_color_height = color_height;
    m_depth_width = depth_width;
    m_depth_height = depth_height;
    m_config.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    m_config.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, 30);
    m_config.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_YUYV, 30);
    m_config.enable_stream(RS2_STREAM_INFRARED, 1, depth_width, depth_height, RS2_FORMAT_Y8, 30);
    m_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    m_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    if (m_record)
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        auto lt = std::put_time(&tm, "%Y%m%d_%H%M%S");
        std::stringstream ss;
        ss << m_record_file_url.toString(QUrl::RemoveScheme).toStdString() << '/' << lt << ".bag";
        std::cout << "Recording to: " << ss.str() << std::endl;
        m_config.enable_record_to_file(ss.str());
    }
    m_align_to_color = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
}

void RealsenseDataProviderLive::start()
{
    m_pipe_profile = m_pipe.start(m_config, [&](const rs2::frame &frame)
    {
        if (auto motion_frame = frame.as<rs2::motion_frame>())
        {
            m_motion_frame = QSharedPointer<rs2::motion_frame>::create(motion_frame);
            emit new_motion_frame(m_motion_frame);
        }
        if (auto frameset = frame.as<rs2::frameset>())
        {
            m_frameset = QSharedPointer<rs2::frameset>::create(frameset);
            emit new_frameset(m_frameset);
        }
    });
}

void RealsenseDataProviderLive::stop()
{
    qDebug() << "Stopping RealsenseDataProviderLive";
}

void RealsenseDataProviderLive::pause()
{
    qDebug() << "Pausing RealsenseDataProviderLive";
}

void RealsenseDataProviderLive::set_color_resolution(QString resolution)
{
    qDebug() << "Setting color resolution to: " << resolution;
}

void RealsenseDataProviderLive::set_depth_resolution(QString resolution)
{
    qDebug() << "Setting depth resolution to: " << resolution;
}

bool RealsenseDataProviderLive::started() const
{
    return m_started;
}

//
//
//  RealsenseImageProvider
//
//

QImage RealsenseImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    return QPixmap(requestedSize.width(), requestedSize.height()).toImage();
}

void RealsenseImageProvider::set_color_image(QSharedPointer<QImage> color_image)
{
    qDebug() << "Image Provider received new Frameset";
}

void RealsenseImageProvider::set_color_intrinsics(QSharedPointer<rs2_intrinsics> color_intrinsics)
{
    m_color_intrinsics = color_intrinsics;
}

void RealsenseImageProvider::set_depth_intrinsics(QSharedPointer<rs2_intrinsics> depth_intrinsics)
{
    m_depth_intrinsics = depth_intrinsics;
}

void RealsenseImageProvider::set_groundplane_cloud(QSharedPointer<pcl::PointCloud<pcl::PointXYZ> > groundplane_cloud)
{
    m_groundplane_cloud = groundplane_cloud;
}

//
//
//  RealsenseManager
//
//

RealsenseManager::RealsenseManager()
{
}

void RealsenseManager::start(int device_index, QString color_resolution, QString depth_resolution)
{
    QRegExp rx("[x]");
    QStringList color_resolution_list = color_resolution.split(rx, Qt::SkipEmptyParts);
    QStringList depth_resolution_list = depth_resolution.split(rx, Qt::SkipEmptyParts);
    if (realsenseDeviceList->use_bag())
    {
        qDebug() << "Starting stream from file: " << realsenseDeviceList->bagFile_url();
    }
    else
    {
        qDebug() << "Starting stream from device: " << realsenseDeviceList->selected_device().get_info(RS2_CAMERA_INFO_NAME) << "with"
                 << color_resolution_list[0] << "x" << color_resolution_list[1];
        dataProviderLive = QSharedPointer<RealsenseDataProviderLive>::create(realsenseDeviceList->selected_device(),
                                                                         color_resolution_list[0].toInt(), color_resolution_list[1].toInt(),
                                                                         depth_resolution_list[0].toInt(), depth_resolution_list[1].toInt());
        connect(dataProviderLive.data(), &RealsenseDataProvider::new_frameset, realsenseImageProvider, &RealsenseImageProvider::set_frameset);
        dataProviderLive->start();
    }
}

void RealsenseManager::stop(int device_index)
{
    if (dataProviderLive) dataProviderLive->stop();
}

void RealsenseManager::pause(int device_index)
{

}

void RealsenseManager::trigger_frameReady()
{
    emit frameReady();
}
