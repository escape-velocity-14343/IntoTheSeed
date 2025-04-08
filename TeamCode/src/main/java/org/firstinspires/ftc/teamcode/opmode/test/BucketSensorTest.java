package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.BucketRelocalizeCommand;
import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@TeleOp
public class BucketSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        BucketSensorSubsystem bucketSensor = new BucketSensorSubsystem(hardwareMap);

        waitForStart();

        pinpoint.resetYaw();
        pinpoint.setPosition(-65, 40);
        BucketRelocalizeCommand bucketCommand = new BucketRelocalizeCommand(bucketSensor, pinpoint, 1.0);
        bucketCommand.setTelemetry(telemetry);
        bucketCommand.setDryRun(true);
        CommandScheduler.getInstance().schedule(bucketCommand);

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("pinpoint x", pinpoint.getPose().getX());
            telemetry.addData("pinpoint y", pinpoint.getPose().getY());
            telemetry.update();
        }
    }
}
