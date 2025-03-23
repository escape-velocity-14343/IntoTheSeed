package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class PNPTest extends LinearOpMode {
    public static boolean red = true;
    VisionSubsystem highCameraSubsystem;
    PinpointSubsystem pinpoint;

    public static double cx = 338.083;
    public static double cy = 218.771;
    public static double focalL = 491.437;
    RobotPnP pnp;

    public static int exposure = 40;

    DcMotor fr, fl, br, bl;

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frontRight");
        fl = hardwareMap.dcMotor.get("frontLeft");
        br = hardwareMap.dcMotor.get("backRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        pnp = new RobotPnP(cx, cy, focalL);

        if (red){
            AutoConstants.alliance = AutoConstants.Alliance.RED;
        }
        else{
            AutoConstants.alliance = AutoConstants.Alliance.BLUE;
        }
        int[] viewportids = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.VERTICAL);
        highCameraSubsystem = new VisionSubsystem(hardwareMap, VisionConstants.chassisCameraName, telemetry, viewportids[0]);
        pinpoint = new PinpointSubsystem(hardwareMap);
        pinpoint.reset();
        pinpoint.resetYaw();

        highCameraSubsystem.waitForSetExposure(3000, 10000, exposure);

        CommandScheduler.getInstance().registerSubsystem(pinpoint, highCameraSubsystem);

        waitForStart();
        while (opModeIsActive()){

            Vector2d samplePos = highCameraSubsystem.getSamplePos();

            Vector2d sampleFCPos = pnp.getFieldCoordinates((int) samplePos.getX(), (int) samplePos.getY(), pinpoint.getPose());

            Translation2d sampleRCPos = pnp.getRobotCentricTranslation((int) samplePos.getX(), (int) samplePos.getY());

            telemetry.addData("Sample X", sampleFCPos.getX());
            telemetry.addData("Sample Y", sampleFCPos.getY());
            telemetry.addData("Sample Robot X", sampleRCPos.getX());
            telemetry.addData("Sample Robot Y", sampleRCPos.getY());
            telemetry.addData("Sample PX", samplePos.getX());
            telemetry.addData("Sample PY", samplePos.getY());

            telemetry.update();

            CommandScheduler.getInstance().run();
        }
    }

}
