package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
@Config
public class ChassisCameraTest extends LinearOpMode {
    public static boolean isRed = true;
    VisionSubsystem HighCameraSubsystem;

    public static int exposure = 40;
    public static int whiteBalance = 4000;

    DcMotor fr, fl, br, bl;

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("frontRight");
        fl = hardwareMap.dcMotor.get("frontLeft");
        br = hardwareMap.dcMotor.get("backRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        HighCameraSubsystem = new VisionSubsystem(hardwareMap, VisionConstants.chassisCameraName, () -> isRed, telemetry);

        HighCameraSubsystem.waitForSetExposure(3000, 10000, exposure);
//        HighCameraSubsystem.waitForSetGain(3000, 10000, gain);
        HighCameraSubsystem.waitForSetWhiteBalance(3000, 10000, whiteBalance);

        waitForStart();
        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);

            CommandScheduler.getInstance().run();
        }
    }

}
