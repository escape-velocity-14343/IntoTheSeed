package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@Config
public class BucketRelocalizeCommand extends CommandBase {
    public static double correctionFactor = 0.956701;
    public static double offsetX = 7.75;
    public static double offsetY = 10.75;
    BucketSensorSubsystem bucketSensor;
    PinpointSubsystem pinpoint;
    Telemetry telemetry;

    public BucketRelocalizeCommand(BucketSensorSubsystem bucketSensor, PinpointSubsystem pinpoint, Telemetry telemetry) {
        this.bucketSensor = bucketSensor;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        //addRequirements(bucketSensor, pinpoint);
    }

    @Override
    public void execute() {
        double theta = pinpoint.getPose().getRotation().getRadians();
        double headingCorrectionFactor = Math.cos(theta + Math.PI / 4.0);
        double xInches = bucketSensor.getSensorLeft() / correctionFactor * headingCorrectionFactor - 72 + offsetX;
        double yInches = 72 - bucketSensor.getSensorRight() / correctionFactor * headingCorrectionFactor - offsetY;
        telemetry.addData("x inches raw", bucketSensor.getSensorLeft());
        telemetry.addData("y inches raw", bucketSensor.getSensorRight());
        telemetry.addData("x inches", xInches);
        telemetry.addData("y inches", yInches);
        telemetry.addData("theta", theta);
        pinpoint.setPosition(xInches, yInches);
        Log.i("bucket relocalize", String.format("relocalized to %s %s", xInches, yInches));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
