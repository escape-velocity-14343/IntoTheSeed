package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

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

        // driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RetractCommand(wrist,
        // pivot, extension));

        configureDriver();
        configureOperator();
        configureDualControl();

        /*new RunCommand(() -> extension.setPower(- gamepad2.left_trigger - gamepad1.left_trigger), extension),
        () -> extension.getCurrentPosition() / SlideConstants.ticksPerInch < SlideConstants.submersibleIntakeMaxExtension))*/

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
            // if (intake.getFrontV()>IntakeConstants.intakeSensorVoltageThres) {
            //    gamepad1.rumble(100);
            // }
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
        driverPad.getGamepadButton(GamepadKeys.Button.X).and(extension.extendedTrigger.negate()).whenActive(bucketPos());

        driverPad
                .getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new RetractCommand(wrist, pivot, extension, turret));

        // ------- INTAKE -------
        driverPad
                .getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new SubPosReadyCommand(
                                extension,
                                pivot,
                                wrist,
                                intake,
                                turret,
                                0,
                                SlideConstants.submersibleIntakeMaxExtension));

        driverPad
                .getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new SubPosReadyCommand(
                                extension,
                                pivot,
                                wrist,
                                intake,
                                turret,
                                90,
                                SlideConstants.submersibleIntakeMaxExtension));

        //        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
        //                new TurretCommand(turret, turret.getPosition()-25)
        //        );
        //        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
        //                new TurretCommand(turret, turret.getPosition()+25)
        //        );

        new Trigger(
                () ->
                        driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
                                && driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                                < 0.6)
                .whileActiveContinuous(
                        new SubPosCommand(
                                extension,
                                wrist,
                                intake,
                                pivot,
                                () -> driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8)
                .whileActiveOnce(subPos())
                .whenInactive(
                        new IntakeRetractCommand(wrist, pivot, extension, turret)
                                .alongWith(
                                        new IntakeControlCommand(
                                                intake, IntakeConstants.closedPos, 0)));

        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8)
                .whileActiveOnce(new IntakeClawCommand(intake, IntakeConstants.openPos))
                .whenInactive(
                        new ConditionalCommand(
                                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                                new IntakeClawCommand(intake, IntakeConstants.closedPos),
                                () -> getState() == FSMStates.INTAKE));
    }

    public void configureOperator() {
        operatorPad
                .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new TurretCommand(turret, () -> turret.getPosition() - 45));
        operatorPad
                .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new TurretCommand(turret, () -> turret.getPosition() + 45));

//        operatorPad
//                .getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(
//                        new SubPosReadyCommand(
//                                extension,
//                                pivot,
//                                wrist,
//                                intake,
//                                turret,
//                                90,
//                                SlideConstants.submersibleIntakeMinExtension));

        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(extension.resetC());

        Trigger leftOperatorTrigger = new Trigger(() -> operatorPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        Trigger rightOperatorTrigger = new Trigger(() -> operatorPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);

        extension.manualControlTrigger.whenActive(extension.openloopC(() -> Util.applyDeadband(operatorPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - operatorPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), SlideConstants.manualControlDeadband)));
        pivot.manualControlTrigger.whenActive(pivot.openloopC(() -> Util.applyDeadband(operatorPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - operatorPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), PivotConstants.manualControlDeadband)));

        //A and Y on opposite sides
        //X and B on opposite sides
        operatorPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(extension.enableManualControl());
        operatorPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(extension.disableManualControl());

        operatorPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(pivot.enableManualControl());
        operatorPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(pivot.disableManualControl());
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
