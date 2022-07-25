import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Layouts 1.12
import QtDataVisualization 1.12

Window {
    width: 640
    height: 480
    title: "Profile View"
    visible: true

    ColumnLayout {
        anchors.fill: parent
        Scatter3D {
            id: scatter
            width: parent.width
            height: parent.height
            Scatter3DSeries {
                ItemModelScatterDataProxy {
                    itemModel: Realsense.points3d
                }
            }
        }
    }
}
