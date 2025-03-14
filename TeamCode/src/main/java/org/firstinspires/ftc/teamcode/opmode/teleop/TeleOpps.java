package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.constants.AutoConstants.autoscoreMaxVel;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.scorePos;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.custom.BasketAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.RunIfCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import javax.crypto.spec.OAEPParameterSpec;

@TeleOp(group = "0", name = "TeleOpp")
@Config
public class TeleOpps extends Robot {
    public static double manualMultiplier = 0.7;
    public static double robotMovementMultiplier = 1.0;

    protected GamepadEx driverPad;
    protected GamepadEx operatorPad;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driverPad = new GamepadEx(gamepad1);
        operatorPad = new GamepadEx(gamepad2);

        if (true) {
            CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                    mecanum,
                    () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftY() / driverPad.getLeftX()) < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftX() / driverPad.getLeftY()) < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> pinpoint.getPose().getRotation().getDegrees()
            ) {
                //@Override
                //public double getXModPower() {
                //    if (getState() != FSMStates.SPECIMEN) {
                //        return 0.0;
                //    }
                //
                //    return
                //}
            });
        } else {
            DefaultDriveCommand drive = new DefaultDriveCommand(mecanum,
                    () -> Util.halfLinearHalfCubic(driverPad.getLeftY() / driverPad.getLeftX() < 0.05 ? 0 : driverPad.getLeftY()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getLeftX() / driverPad.getLeftY() < 0.05 ? 0 : driverPad.getLeftX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * (getState() == FSMStates.INTAKE || getState() == FSMStates.OUTTAKE ? robotMovementMultiplier : 1),
                    () -> 0.0);
            CommandScheduler.getInstance().setDefaultCommand(mecanum, drive);
        }
        //driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RetractCommand(wrist, pivot, extension));


        configureDriver();
        configureOperator();
        configureDualControl();

        /*new RunCommand(() -> extension.setPower(- gamepad2.left_trigger - gamepad1.left_trigger), extension),
        () -> extension.getCurrentPosition() / SlideConstants.ticksPerInch < SlideConstants.submersibleIntakeMaxExtension))*/

        // pinpoint.resetYaw()));
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("motorpos", extension.getCurrentInches());
            telemetry.addData("pivotpos", pivot.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            telemetry.addData("hi", CommandScheduler.getInstance().getDefaultCommand(mecanum));
            telemetry.addData("intake flipped?", reverseClaw.get());
            telemetry.addData("intake front voltage", intake.getFrontV());
            telemetry.addData("intake back voltage", intake.getBackV());
            telemetry.addData("pose x", pinpoint.getPose().getX());
            telemetry.addData("pose y", pinpoint.getPose().getY());
            telemetry.addData("pose heading", pinpoint.getPose().getRotation().getDegrees());
            //if (intake.getFrontV()>IntakeConstants.intakeSensorVoltageThres) {
            //    gamepad1.rumble(100);
            //}
            timer.reset();
            update();
        }
        CommandScheduler.getInstance().reset();
    }

    public void configureDriver() {
        // ------- UTILITIES -------
        // heading reset
        new Trigger(() -> gamepad1.options).whileActiveOnce(new InstantCommand(pinpoint::resetYaw));

        // ------- BUCKET --------
        driverPad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(bucketPos());

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new RetractCommand(wrist, pivot, extension, turret)
        );

        // ------- INTAKE -------
        new Trigger(() -> gamepad1.right_trigger > 0.01).whileActiveOnce(
               new ScheduleCommand(new SubPosReadyCommand(extension, pivot, wrist, intake, turret, 0, SlideConstants.submersibleIntakeMaxExtension))
        );

        new Trigger(() -> gamepad1.left_trigger > 0.01).whileActiveOnce(
                new ScheduleCommand(new SubPosReadyCommand(extension, pivot, wrist, intake, turret, 90, SlideConstants.submersibleIntakeMaxExtension))
        );

        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                        subPos().andThen(
                                new WaitCommand(400),
                                new IntakeRetractCommand(wrist, pivot, extension, turret)
                        )
        ).whenReleased(
                new IntakeRetractCommand(wrist, pivot, extension, turret).alongWith(
                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0)
                )
        );

        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeClawCommand(intake, IntakeConstants.openPos))
                .whenReleased(
                        new ConditionalCommand(
                                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                                new IntakeClawCommand(intake, IntakeConstants.closedPos),
                                () ->  getState() == FSMStates.INTAKE));


    }

    public void configureOperator() {
    }

    public void configureDualControl() {
    }
}

//  ______________
// ||            ||
// ||            ||
// ||____________||
// |______________|
// \\#####EV#####\\
//  \\############\\
//   \      ____    \
//    \_____\___\____\