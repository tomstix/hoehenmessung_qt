import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Window {
    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: qsTr("Realsense Height Measurement")

    property var statusBarSize: 150

    GridLayout {
        anchors.fill: parent
        id: mainGrid
        rows: 1
        columns: 2
        ColumnLayout {
            id: controlLayout
            Layout.preferredWidth: statusBarSize
            ComboBox {
                id: resolutionSelector
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                currentIndex: 4
                model: ListModel {
                    ListElement {text: "424x240"}
                    ListElement {text: "480x270"}
                    ListElement {text: "640x360"}
                    ListElement {text: "640x480"}
                    ListElement {text: "1280x720"}
                }
                Component.onCompleted: Realsense.resolution = currentIndex
                onCurrentIndexChanged: Realsense.resolution = currentIndex
            }
            Button {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                text: "Start"
                onClicked: {
                    Realsense.start()
                }
            }
            Button {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                text: "Stop"
                onClicked: {
                    Realsense.stop()
                }
            }
            Label {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                property string textc: "Nothing received yet"
                id: label
                text: textc
            }
        }

        Image {
            id: image
            source: "image://color"
            Connections {
                target: Realsense
                property int counter
                function onColorImageReady() {
                    image.source = "image://color/" + counter
                    counter = counter + 1
                }
                function onResolutionChanged() {
                    image.fillMode = Image.PreserveAspectFit
                    mainWindow.height = Realsense.height
                    mainWindow.width = Realsense.width + statusBarSize
                }
            }
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: sourceSize.width
            Layout.preferredHeight: sourceSize.height
        }
    }
}
