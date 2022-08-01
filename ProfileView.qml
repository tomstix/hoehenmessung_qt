import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.15
import QtDataVisualization 1.15
import Point3DList 1.0

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
                baseColor: "red"
                ItemModelScatterDataProxy {
                    itemModel: Realsense.groundPlanePoints3D
                    xPosRole: "x"
                    yPosRole: "y"
                    zPosRole: "z"
                }
            }
            Scatter3DSeries {
                ItemModelScatterDataProxy {
                    itemModel: Realsense.restPoints3D
                    xPosRole: "x"
                    yPosRole: "y"
                    zPosRole: "z"
                }
            }
        }
    }
    onClosing: profileViewLoader.source = ""
}
