package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
@Config
public class ChassisCameraTest extends LinearOpMode {
    public static boolean red = true;
    VisionSubsystem highCameraSubsystem;

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

        if (red){
            AutoConstants.alliance = AutoConstants.Alliance.RED;
        }
        else{
            AutoConstants.alliance = AutoConstants.Alliance.BLUE;
        }
        highCameraSubsystem = new VisionSubsystem(hardwareMap, VisionConstants.chassisCameraName, telemetry);

        highCameraSubsystem.waitForSetExposure(3000, 10000, exposure);

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
