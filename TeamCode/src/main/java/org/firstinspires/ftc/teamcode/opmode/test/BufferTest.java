package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.InterpolatableBuffer;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@TeleOp
public class BufferTest extends LinearOpMode {

    PinpointSubsystem pinpoint;
    InterpolatableBuffer buffer;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fr, fl, br, bl;
        fr = hardwareMap.dcMotor.get("frontRight");
        fl = hardwareMap.dcMotor.get("frontLeft");
        br = hardwareMap.dcMotor.get("backRight");
        bl = hardwareMap.dcMotor.get("backLeft");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);


        pinpoint = new PinpointSubsystem(hardwareMap);
        buffer = new InterpolatableBuffer(pinpoint, 5000.0);

        pinpoint.reset();
        pinpoint.resetYaw();

        waitForStart();

        while (!isStopRequested()){
            telemetry.addData("pinpoint pose", pinpoint.getPose().toString());
            telemetry.addData("2s ago pose", buffer.getPastSample(2000.0).toString());
            CommandScheduler.getInstance().run();
            telemetry.update();

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
        }
    }
}

