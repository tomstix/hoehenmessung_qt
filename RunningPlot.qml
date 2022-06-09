import QtQuick 2.15
import QtCharts 2.15

ChartView {
    antialiasing: true

    property real yMin: 0
    property real yMax: 0
    property real xMin: 0
    property real xMax: 0

    ValueAxis {
        id: xAxis
        min: 0
        max: 60
    }
    ValueAxis {
        id: yAxis
        min: -3.0
        max: 3.0
    }

    MouseArea {
        anchors.fill: parent
        onPressAndHold: {
            lineSeries.removePoints(0, lineSeries.count)
            yMin = 0
            yMax = 0
        }
    }

    Connections {
        target: Realsense
        function onNewHeightPoint() {
            lineSeries.append(Realsense.heightPoint.x / 1000.0, Realsense.heightPoint.y)
            xAxis.min = Realsense.heightPoint.x / 1000.0 - 60.0
            xAxis.max = Realsense.heightPoint.x / 1000.0 + 1.0
            lineSeries.pointsVisible = false
            lineSeries.pointsVisible = true
            if (Realsense.heightPoint.x / 1000.0 > xMax) xMax = Realsense.heightPoint.x / 1000.0
            if (Realsense.heightPoint.x / 1000.0 < xMin) xMin = Realsense.heightPoint.x / 1000.0
            if (Realsense.heightPoint.y > yMax) yMax = Realsense.heightPoint.y
            if (Realsense.heightPoint.y < yMin) yMin = Realsense.heightPoint.y
            yAxis.min = yMin - 0.1
            yAxis.max = yMax + 0.1
        }
        function onRunningChanged() {
            if (!Realsense.running)
            {
                lineSeries.removePoints(0, lineSeries.count)
            }
        }
    }

    LineSeries {
        id: lineSeries
        name: "Measured Height"
        axisX: xAxis
        axisY: yAxis
    }
}