package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PtoSubsystem;

@TeleOp
public class PtoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CachingVoltageSensor voltage = new CachingVoltageSensor(hardwareMap);

        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);

        MecanumDriveSubsystem mecanum =
                new MecanumDriveSubsystem(
                        "frontRight",
                        "frontLeft",
                        "backRight",
                        "backLeft",
                        hardwareMap,
                        pinpoint,
                        voltage);
        PtoSubsystem pto = new PtoSubsystem(hardwareMap);
        waitForStart();

        while (!isStopRequested()) {
            pto.setEngaged(gamepad1.right_bumper);
            mecanum.driveRaw(
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_y
            );
        }
    }
}
