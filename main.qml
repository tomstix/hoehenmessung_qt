import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import Realsense 1.0

Window {
    Realsense {
        id: rs
        }

    Connections {
    }
    width: 640
    height: 480
    visible: true
    title: qsTr("Realsense Height Measurement")

    Column {
        ComboBox {
            id: resolutionSelector
            currentIndex: 4
            model: ListModel {
                ListElement {text: "424x240"}
                ListElement {text: "480x270"}
                ListElement {text: "640x360"}
                ListElement {text: "640x480"}
                ListElement {text: "1280x720"}
                ListElement {text: "1280x800"}
            }
            Component.onCompleted: rs.resolution = currentIndex
            onCurrentIndexChanged: rs.resolution = currentIndex
        }
        Button {
            text: "Start"
            onClicked: {
                rs.start()
            }
        }
        Button {
            text: "Stop"
            onClicked: {
                rs.stop()
            }
        }
        Label {
            property var textc: "Nothing received yet"
            
            id: label
            text: textc
        }
    }
}