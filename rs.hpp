#pragma once

#include <iostream>

#include <QObject>
#include <QQuickImageProvider>
#include <QImage>
#include <QPixmap>
#include <QAbstractListModel>
#include <QString>

#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

class RealsenseDeviceList : public QAbstractListModel
{
    Q_OBJECT
public:
    RealsenseDeviceList(QObject *parent = nullptr);
    enum RealsenseDeviceRoles
    {
        RS2_CAMERA_INFO_NAME_ROLE = Qt::UserRole,
        RS2_CAMERA_INFO_SERIAL_NUMBER_ROLE,
        RS2_CAMERA_INFO_FIRMWARE_VERSION_ROLE,
        RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION_ROLE,
        RS2_CAMERA_INFO_PHYSICAL_PORT_ROLE,
        RS2_CAMERA_INFO_DEBUG_OP_CODE_ROLE,
        RS2_CAMERA_INFO_ADVANCED_MODE_ROLE,
        RS2_CAMERA_INFO_PRODUCT_ID_ROLE,
        RS2_CAMERA_INFO_CAMERA_LOCKED_ROLE,
        RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR_ROLE,
        RS2_CAMERA_INFO_PRODUCT_LINE_ROLE,
        RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER_ROLE,
        RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID_ROLE,
        RS2_CAMERA_INFO_IP_ADDRESS_ROLE,
        RS2_CAMERA_INFO_COUNT_ROLE
    };
    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;

    const QUrl &bagFile_url() const;
    void setBagFile_url(const QUrl &newBagFile_url);

    const rs2::device &selected_device() const;
    void setSelected_device(const rs2::device &newSelected_device);

    bool use_bag() const;
    void setUse_bag(bool newUse_bag);

    QStringList color_resolutions() const;
    QStringList depth_resolutions() const;

signals:
    void bagFile_urlChanged();
    void selected_deviceChanged();
    void use_bagChanged();
    void color_resolutionsChanged();
    void depth_resolutionsChanged();

public slots:
    void set_device(int index);

protected:
    QHash<int, QByteArray> roleNames() const;

private:
    rs2::context m_ctx;
    QVector<rs2::device> m_devices;
    QUrl m_bagFile_url;
    Q_PROPERTY(QUrl bagFile_url READ bagFile_url WRITE setBagFile_url NOTIFY bagFile_urlChanged)
    rs2::device m_selected_device;
    Q_PROPERTY(rs2::device selected_device READ selected_device NOTIFY selected_deviceChanged)
    bool m_use_bag = false;
    Q_PROPERTY(bool use_bag READ use_bag NOTIFY use_bagChanged)
    void refresh();
    QStringList m_color_resolutions;
    Q_PROPERTY(QStringList color_resolutions READ color_resolutions NOTIFY color_resolutionsChanged)
    QStringList m_depth_resolutions;
    Q_PROPERTY(QStringList depth_resolutions READ depth_resolutions NOTIFY depth_resolutionsChanged)
};

class RealsenseDataProvider : public QObject
{
    Q_OBJECT
public:
    RealsenseDataProvider(QObject *parent = nullptr);
public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void pause() = 0;
signals:
    void color_resolutionChanged();
    void depth_resolutionChanged();
    void new_frameset(QSharedPointer<rs2::frameset> frameset);
    void new_motion_frame(QSharedPointer<rs2::motion_frame> motion_frame);
protected:
    int m_color_width = 640;
    int m_color_height = 480;
    int m_depth_width = 640;
    int m_depth_height = 480;
    QSharedPointer<rs2::frameset> m_frameset;
    QSharedPointer<rs2::motion_frame> m_motion_frame;
    rs2_intrinsics m_color_intrinsics;
    rs2_intrinsics m_depth_intrinsics;
    rs2::pipeline m_pipe;
    rs2::pipeline_profile m_pipe_profile;
    rs2::config m_config;
    std::unique_ptr<rs2::align> m_align_to_color;
};

class RealsenseDataProviderLive : public RealsenseDataProvider
{
    Q_OBJECT
public:
    RealsenseDataProviderLive();
    RealsenseDataProviderLive(rs2::device device, int color_width, int color_height, int depth_width, int depth_height);
    bool started() const;

public slots:
    void start() override;
    void stop() override;
    void pause() override;
    void set_color_resolution(QString resolution);
    void set_depth_resolution(QString resolution);
private:
    rs2::device m_device;
    bool m_record = false;
    QUrl m_record_file_url;
    void m_new_frame_callback(const rs2::frame& frame);
    bool m_started;
};

class RealsenseImageProvider : public QObject, public QQuickImageProvider
{
    Q_OBJECT
public:
    RealsenseImageProvider() : QQuickImageProvider(QQuickImageProvider::Image){}
    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
public slots:
    void set_color_image(QSharedPointer<QImage> color_image);
    void set_color_intrinsics(QSharedPointer<rs2_intrinsics> color_intrinsics);
    void set_depth_intrinsics(QSharedPointer<rs2_intrinsics> depth_intrinsics);
    void set_groundplane_cloud(QSharedPointer<pcl::PointCloud<pcl::PointXYZ>> groundplane_cloud);
signals:
    void frameReady();
private:
    QSharedPointer<QImage> m_color_image;
    QSharedPointer<rs2_intrinsics> m_color_intrinsics;
    QSharedPointer<rs2_intrinsics> m_depth_intrinsics;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_groundplane_cloud;
};

class RealsenseManager : public QObject
{
    Q_OBJECT
public:
    RealsenseManager();
    RealsenseDeviceList* realsenseDeviceList = new RealsenseDeviceList;
    RealsenseImageProvider* realsenseImageProvider = new RealsenseImageProvider;
    QSharedPointer<RealsenseDataProviderLive> dataProviderLive;
public slots:
    void start(int device_index, QString color_resolution, QString depth_resolution);
    void stop(int device_index);
    void pause(int device_index);
    void trigger_frameReady();
signals:
    void frameReady();
};
