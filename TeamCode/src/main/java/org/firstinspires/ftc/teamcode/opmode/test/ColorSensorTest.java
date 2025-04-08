package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.drivers.VEML3328;

@TeleOp(group = "test")
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VEML3328 colorSensor = hardwareMap.get(VEML3328.class, "color");

        waitForStart();

        while (!isStopRequested()){
            telemetry.addData("Red value:", colorSensor.getColor().toString());
            telemetry.update();
        }
    }
}
