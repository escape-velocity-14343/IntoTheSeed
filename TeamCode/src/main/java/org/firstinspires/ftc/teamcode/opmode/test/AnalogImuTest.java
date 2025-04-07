package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(group = "test")
public class AnalogImuTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput imu = hardwareMap.get(AnalogInput.class, "clawImu");
        AnalogInput prox = hardwareMap.get(AnalogInput.class, "clawProx");

        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("imu voltage", imu.getVoltage());
            telemetry.addData("prox voltage", prox.getVoltage());
            telemetry.update();
        }
    }
}
