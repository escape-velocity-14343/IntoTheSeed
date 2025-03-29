package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class PinpointTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        GamepadEx driverPad = new GamepadEx(gamepad1);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        pinpoint.reset();
        pinpoint.resetYaw();

        CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                mecanum,
                () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftY() / driverPad.getLeftX()) < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? 1 : 1),
                () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftX() / driverPad.getLeftY()) < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? 1 : 1),
                () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? 1 : 1),
                () -> pinpoint.getPose().getRotation().getDegrees()));


        waitForStart();
        pinpoint.setPosition(-65, 40);

        while (opModeIsActive()) {
            update();
            telemetry.addData("x", pinpoint.getPose().getX());
            telemetry.addData("y", pinpoint.getPose().getY());
            telemetry.addData("heading", pinpoint.getPose().getRotation().getDegrees());
            telemetry.addData("imu heading", imu.getRobotYawPitchRollAngles().getYaw());
            int[] enc = pinpoint.getEncoderCounts();
            telemetry.addData("x enc", enc[0]);
            telemetry.addData("y enc", enc[1]);
        }
    }
}
