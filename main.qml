import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtCharts 2.15
import QtQuick.Dialogs 1.3

ApplicationWindow {
    id: mainWindow
    width: 1050
    height: 630
    minimumHeight: 550
    visible: true
    visibility: "Maximized"
    title: qsTr("Realsense Height Measurement")

    property bool shallClose: false

    onClosing: {
        close.accepted = !Realsense.running
        shallClose = true
        Realsense.stop()
    }

    Loader {
        id: canSettingsLoader
    }

    menuBar: MenuBar {
        Menu {
            title: qsTr("&Settings")
            MenuItem {
                action: openCANSettings
            }
            Action {
                text: qsTr("&Open...")
                onTriggered: filePicker.visible = true
            }
            MenuSeparator { }
            Action {
                text: qsTr("&Quit")
                onTriggered: mainWindow.close()
            }
        }
    }

    Action {
        id: openCANSettings
        text: qsTr("&CAN Bus Settings")
        onTriggered: {
            canSettingsLoader.source = "CANSettings.qml"
        }
    }

    FileDialog {
        id: filePicker
        title: "Please choose a .bag file"
        folder: shortcuts.desktop
        nameFilters: ["BAG Files (*.bag)", "All Files (*)"]
        onAccepted: {
            Realsense.bagFile = filePicker.fileUrl
            close()
        }
        onRejected: {
            close()
        }
    }

    Connections {
        target: Realsense
        function onIsRunningChanged()
        {
            if (shallClose)
            {
                Qt.quit();
            }
        }
    }
    GridLayout {
        anchors.fill: parent
        id: mainGrid
        rows: 1
        columns: 2
        Connections {
            target: mainWindow
            function onWidthChanged()
            {
                sideBar.Layout.preferredWidth = mainWindow.width / 5
            }
            function onHeightChanged()
            {
                sideBar.Layout.maximumHeight = mainWindow.height
                image.Layout.maximumHeight = mainWindow.height
            }
        }

        ScrollView {
            id: sideBar
            clip: true
            Layout.minimumWidth: 200
            Layout.maximumWidth: 500
            Layout.fillWidth: true
            Layout.fillHeight: true
            Column {
                id: scrollViewLayout
                Controls {
                    id: controls
                    width: sideBar.width
                }
                RunningPlot {
                    id: heightPlot
                    width: sideBar.width
                    height: 400
                }
            }
        }

        Image {
            id: image
            Layout.column: 1
            property string src: "image://realsense/color/"
            source: src + 0
            fillMode: Image.PreserveAspectFit
            Connections {
                target: Realsense
                property int counter: 0
                function onNewFrameReady()
                {
                    image.source = image.src + counter
                    counter = !counter
                }
                function onHeightChanged()
                {
                    image.Layout.preferredWidth = Realsense.width
                    image.Layout.preferredHeight = Realsense.height
                }
            }
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}
