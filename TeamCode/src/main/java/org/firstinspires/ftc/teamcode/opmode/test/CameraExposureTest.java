package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.custom.ExposureCycle;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "Camera Exposure Test", group = "Test")
public class CameraExposureTest extends Robot {
    VisionSubsystem cam;

    @Override
    public void runOpMode() throws InterruptedException {
        cam = new VisionSubsystem(hardwareMap, telemetry);

        initialize();
        cam.waitForSetExposure(5000, 5000, 10);

        waitForStart();

        cs.schedule(
                new ExposureCycle(cam, true),
                new ExposureCycle(cam, false)
        );


        while (!isStopRequested()) {
            update();
        }
        cs.reset();
    }
}