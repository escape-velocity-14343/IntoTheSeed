package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@Config
public class BucketRelocalizeCommand extends CommandBase {
    public static double correctionFactor = 0.956701;
    public static double offsetX = 7.75;
    public static double offsetY = 10.75;
    BucketSensorSubsystem bucketSensor;
    PinpointSubsystem pinpoint;

    public BucketRelocalizeCommand(BucketSensorSubsystem bucketSensor, PinpointSubsystem pinpoint) {
        this.bucketSensor = bucketSensor;
        this.pinpoint = pinpoint;
        //addRequirements(bucketSensor, pinpoint);
    }

    @Override
    public void execute() {
        double theta = pinpoint.getPose().getRotation().getRadians();
        double headingCorrectionFactor = Math.cos(theta + Math.PI / 4.0);
        double xInches = bucketSensor.getSensorLeft() / correctionFactor * headingCorrectionFactor - 72 + offsetX;
        double yInches = 72 - bucketSensor.getSensorRight() / correctionFactor * headingCorrectionFactor - offsetY;
        pinpoint.setPosition(xInches, yInches);
        Log.i("bucket relocalize", String.format("relocalized to %s %s", xInches, yInches));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
