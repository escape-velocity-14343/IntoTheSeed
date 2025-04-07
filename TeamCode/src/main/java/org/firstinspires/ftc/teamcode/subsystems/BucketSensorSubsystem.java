package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedList;
import java.util.Queue;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Util;

@Config
public class BucketSensorSubsystem extends SubsystemBase {
    public static int rollingAverageSize = 5;

    private final AnalogInput sensorLeft;
    private final AnalogInput sensorRight;
    private DistanceUnit unit = DistanceUnit.INCH;

    private final Queue<Double> sensorLeftData = new LinkedList<>();
    private final Queue<Double> sensorRightData = new LinkedList<>();

    private double sensorLeftAverage;
    private double sensorRightAverage;

    public BucketSensorSubsystem(HardwareMap hardwareMap) {
        sensorLeft = hardwareMap.get(AnalogInput.class, "bucketSensorLeft");
        sensorRight = hardwareMap.get(AnalogInput.class, "bucketSensorRight");
    }

    public void setDistanceUnit(DistanceUnit distanceUnit) {
        unit = distanceUnit;
    }

    /**
     * @return In whatever unit you set it to
     */
    public double getSensorLeft() {
        return unit.fromCm(sensorLeft.getVoltage() * 500 / 3.3);
    }

    /**
     * @return In whatever unit you set it to
     */
    public double getSensorRight() {
        return unit.fromCm(sensorRight.getVoltage() * 500 / 3.3);
    }

    @Override
    public void periodic() {
        sensorLeftData.add(sensorLeft.getVoltage());
        if (sensorLeftData.size() > rollingAverageSize) {
            sensorLeftData.remove();
        }

        sensorLeftAverage = Util.median(sensorLeftData);

        sensorRightData.add(sensorRight.getVoltage());
        if (sensorRightData.size() > rollingAverageSize) {
            sensorRightData.remove();
        }

        sensorRightAverage = Util.median(sensorRightData);
    }
}
