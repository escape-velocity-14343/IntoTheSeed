package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "1")
public class DrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor fr, fl, br, bl;
        fr = hardwareMap.dcMotor.get("frontRight");
        fl = hardwareMap.dcMotor.get("frontLeft");
        br = hardwareMap.dcMotor.get("backRight");
        bl = hardwareMap.dcMotor.get("backLeft");

        waitForStart();

        while (opModeIsActive()) {
            fr.setPower(-gamepad1.left_stick_y);
            fl.setPower(gamepad1.left_stick_x);

            br.setPower(-gamepad1.right_stick_y);
            bl.setPower(gamepad1.right_stick_x);
        }
    }
}
