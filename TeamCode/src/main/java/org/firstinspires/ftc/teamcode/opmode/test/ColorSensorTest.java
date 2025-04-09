package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.drivers.VEML3328;

import java.io.Console;

@TeleOp(group = "test")
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VEML3328 colorSensor = hardwareMap.get(VEML3328.class, "colorSensor");

        waitForStart();

        Log.i("i2c class: ", colorSensor.getClass().getName());

        while (!isStopRequested()){
//            telemetry.addData("Red value:", colorSensor.getColor().red);
//            telemetry.addData("Green value:", colorSensor.getColor().green);
//            telemetry.addData("Blue value:", colorSensor.getColor().blue);


            telemetry.update();
        }
    }
}
