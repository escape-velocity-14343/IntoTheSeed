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
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.function.DoubleSupplier;

@TeleOp(group = "0", name = "TeleOpp")
@Config
public class TeleOpps extends Robot {
    public static double manualMultiplier = 0.7;
    public static double robotMovementMultiplier = 1.0;

    double extendInches = SlideConstants.submersibleIntakeMidExtension;

    protected GamepadEx driverPad;
    protected GamepadEx operatorPad;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driverPad = new GamepadEx(gamepad1);
        operatorPad = new GamepadEx(gamepad2);

        DoubleSupplier fieldCentricHeading = true ? () -> pinpoint.getPose().getRotation().getDegrees() : () -> 0.0;
        DoubleSupplier xyGain = () -> currentlyInState(FSMStates.INTAKE, FSMStates.INTAKE_READY) ? 0.7 : 1;
        DoubleSupplier tGain = () -> currentlyInState(FSMStates.INTAKE, FSMStates.INTAKE_READY) ? 0.3 : 1;
        CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                mecanum,
                () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftY() / driverPad.getLeftX()) < 0.05 ? 0 : driverPad.getLeftY()) * xyGain.getAsDouble(),
                () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftX() / driverPad.getLeftY()) < 0.05 ? 0 : driverPad.getLeftX()) * xyGain.getAsDouble(),
                () -> Util.halfLinearHalfCubic(driverPad.getRightX()) * tGain.getAsDouble(),
                fieldCentricHeading
        ));


        configureDriver();
        configureOperator();
        configureDualControl();


        waitForStart();
        // temporary pls remove later
        //pinpoint.setPosition(-65, 40);
        while (!isStopRequested()) {
            telemetry.addData("current state", getState().toString());
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

            timer.reset();
            update();
        }
        CommandScheduler.getInstance().reset();
        intake.setClawer(IntakeConstants.singleIntakePos);
    }

    public void configureDriver() {
        // ------- UTILITIES -------
        // heading reset
        new Trigger(() -> gamepad1.options && gamepad1.share).whileActiveOnce(new InstantCommand(pinpoint::resetYaw));

        // ------- BUCKET --------
        driverPad.getGamepadButton(GamepadKeys.Button.X).whenActive(new ConditionalCommand(
                retract().andThen(bucketPos()),
                bucketPos(),
                inState(FSMStates.INTAKE, FSMStates.INTAKE_READY)
        ));

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(retract());

        // ------- INTAKE -------
        driverPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(intakeReady(() -> 0, () -> extendInches));

        driverPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(intakeReady(() -> 90, () -> extendInches));

        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(new InstantCommand(() -> extendInches = SlideConstants.submersibleIntakeMidExtension));
        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(new InstantCommand(() -> extendInches = SlideConstants.submersibleIntakeMaxExtension));

        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(intake(() -> extendInches)).whenReleased(intakeReady(() -> lastTurretAngle, () -> extendInches));

        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeControlCommand(intake, IntakeConstants.openPos, 0))
                .whenReleased(new ConditionalCommand(
                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                        inState(FSMStates.INTAKE)
                ));

        new Trigger(() -> driverPad.gamepad.touchpad).whenActive(bucketAlign());
    }

    public void configureOperator() {
        operatorPad
                .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new TurretCommand(turret, () -> turret.getPosition() - 45));
        operatorPad
                .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new TurretCommand(turret, () -> turret.getPosition() + 45));


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
