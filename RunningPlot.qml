import QtQuick 2.15
import QtCharts 2.15

ChartView {
    width: 400
    height: 300
    antialiasing: true

    ValueAxis {
        id: xAxis
        min: 0
        max: 500
    }
    ValueAxis {
        id: yAxis
        min: -3.0
        max: 3.0
    }

    Connections {
        target: Realsense
        function onNewHeightPoint() {
            lineSeries.append(Realsense.heightPoint.x / 1000.0, Realsense.heightPoint.y)
            xAxis.min = Realsense.heightPoint.x / 1000.0 - 100.0
            xAxis.max = Realsense.heightPoint.x / 1000.0 + 1.0
            
            lineSeries.pointsVisible = false
            lineSeries.pointsVisible = true
        }
    }

    LineSeries {
        id: lineSeries
        name: "Measured Height"
        axisX: xAxis
        axisY: yAxis
    }
}