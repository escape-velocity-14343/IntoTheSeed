package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
@Config
public class PNPTest extends LinearOpMode {
    public static boolean red = true;
    VisionSubsystem vision;
    PinpointSubsystem pinpoint;

    public static double cx = 338.083 / 2;
    public static double cy = 218.771 / 2;
    public static double focalL = 491.437 / 2;

    public static double x = 0;
    public static double y = 0;
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
        // vision = new VisionSubsystem(hardwareMap, telemetry);
        vision.setCam(true);
        pinpoint = new PinpointSubsystem(hardwareMap);
        pinpoint.reset();
        pinpoint.resetYaw();

        vision.waitForSetExposure(3000, 10000, exposure);

        CommandScheduler.getInstance().registerSubsystem(pinpoint, vision);

        waitForStart();
        pinpoint.setPosition(x, y);

        while (opModeIsActive()) {

            if (gamepad1.x) {
                vision.setCam(false);
            }
            if (gamepad1.y) {
                vision.setCam(true);
            }

            Pose2d samplePos = vision.getSamplePose();

            Translation2d sampleFCPos = pnp.getFieldCoordinates((int) samplePos.getX(), (int) samplePos.getY(), pinpoint.getPose());

            Translation2d sampleRCPos = pnp.getRobotCentricTranslation((int) samplePos.getX(), (int) samplePos.getY());

            telemetry.addData("Sample X", sampleFCPos.getX());
            telemetry.addData("Sample Y", sampleFCPos.getY());
            telemetry.addData("Sample Robot X", sampleRCPos.getX());
            telemetry.addData("Sample Robot Y", sampleRCPos.getY());
            telemetry.addData("Sample PX", samplePos.getX());
            telemetry.addData("Sample PY", samplePos.getY());
            telemetry.addData("Sample Heading", samplePos.getRotation().getDegrees());

            telemetry.update();

            CommandScheduler.getInstance().run();

        }
        CommandScheduler.getInstance().reset();
    }

}
