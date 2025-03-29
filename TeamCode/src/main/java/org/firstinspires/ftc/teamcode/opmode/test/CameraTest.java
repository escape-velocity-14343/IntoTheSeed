package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;

@TeleOp(group="test")
public class CameraTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        int[] viewportids = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);


        CameraName slideCam = hardwareMap.get(WebcamName.class, "slide");
        CameraName chassisCam = hardwareMap.get(WebcamName.class, "chassis");

        VisionPortal vp1 = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(320, 240))
                .setCamera(slideCam)
                .setLiveViewContainerId(viewportids[0])
                .build();

        VisionPortal vp2 = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(320, 240))
                .setCamera(chassisCam)
                .setLiveViewContainerId(viewportids[1])
                .build();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("viewports", viewportids[0] + " " + viewportids[1]);
            telemetry.update();
        }

    }

}
