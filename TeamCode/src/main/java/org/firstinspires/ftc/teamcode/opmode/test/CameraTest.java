package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group="test")
public class CameraTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        int[] viewportids = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        CameraName slideCam = hardwareMap.get(WebcamName.class, "slidecamera");
        CameraName chassisCam = hardwareMap.get(WebcamName.class, "chassiscamera");

        VisionPortal vp1 = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480))
                .setCamera(slideCam)
                .setLiveViewContainerId(viewportids[0])
                .build();

        VisionPortal vp2 = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480))
                .setCamera(chassisCam)
                .setLiveViewContainerId(viewportids[1])
                .build();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("omfg bro wtf", "bruh");
            telemetry.update();
        }

    }

}
