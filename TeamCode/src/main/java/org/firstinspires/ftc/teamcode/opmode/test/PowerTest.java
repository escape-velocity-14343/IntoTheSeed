package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Power Test", group = "Test")
public class PowerTest extends LinearOpMode {
    double maxSpeed = 28*5900/60.0; // Max speed in ticks per second (5900 RPM / 60s/m * 28 ticks per revolution)
    @Override
    public void runOpMode() throws InterruptedException {
        double power = 0.5; // Set the desired power level

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BR");
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        while (opModeInInit()) {
            if(gamepad1.dpadUpWasPressed()) {
                power += 0.05; // Increase power by 0.05
            } else if(gamepad1.dpadDownWasPressed()) {
                power -= 0.05; // Decrease power by 0.05
            }
            telemetry.addData("Power", power);
            telemetry.update();
        }

        while (opModeIsActive()) {
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            double powerIn = batteryVoltageSensor.getVoltage() * (
                    frontLeft.getCurrent(CurrentUnit.AMPS)+
                    backLeft.getCurrent(CurrentUnit.AMPS)+
                    frontRight.getCurrent(CurrentUnit.AMPS)+
                    backRight.getCurrent(CurrentUnit.AMPS)); // Calculate power input based on battery voltage and motor currents
            double velocity = (Math.abs(frontLeft.getVelocity()) +
                    Math.abs(backLeft.getVelocity()) +
                    Math.abs(frontRight.getVelocity()) +
                    Math.abs(backRight.getVelocity()))/4.0; // Average velocity of the motors
            double angularVelocity = 2*Math.PI * velocity / 28; // Convert ticks/s to radians/s (28 ticks per revolution)
            double calculatedTorque = 0.19 * (1 - velocity/(maxSpeed)) - 0.01; // Calculate torque based on motor velocity
            double powerOut = angularVelocity * calculatedTorque; // Calculate power output based on angular velocity and torque


            if(gamepad1.dpadUpWasPressed()) {
                power += 0.05; // Increase power by 0.05
            } else if(gamepad1.dpadDownWasPressed()) {
                power -= 0.05; // Decrease power by 0.05
            }
            power = Range.clip(power, 0, 1); // Ensure power is between 0 and 1
            telemetry.addData("Power", power);

            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.addData("Back Left Power", backLeft.getPower());
            telemetry.addData("Front Right Power", frontRight.getPower());
            telemetry.addData("Back Right Power", backRight.getPower());
            telemetry.addData("Angular Velocity (rad/s)", angularVelocity);
            telemetry.addData("Torque (Nm)", calculatedTorque);
            telemetry.addData("Power Input (W)", powerIn);
            telemetry.addData("Power Output (W)", powerOut);
            telemetry.addData("Velocity (ticks/s)", velocity);
            telemetry.addData("Efficiency (%)", (powerOut / powerIn) * 100);
            telemetry.update();


        }

    }
}
