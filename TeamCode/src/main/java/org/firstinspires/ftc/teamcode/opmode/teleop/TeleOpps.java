package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.FullIntakeFoldCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

@TeleOp(group = "0", name = "TeleOpp")
@Config
public class TeleOpps extends Robot {
    enum IntakeState {
        TOP,
        GROUND,
    }

    public static double manualMultiplier = 0.7;
    public static double robotMovementMultiplier = 1.0;

    double extendInches = SlideConstants.submersibleIntakeMidExtension;
    IntakeState intakeState = IntakeState.TOP;

    protected GamepadEx driverPad;
    protected GamepadEx operatorPad;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driverPad = new GamepadEx(gamepad1);
        operatorPad = new GamepadEx(gamepad2);

        DoubleSupplier fieldCentricHeading = true ? () -> pinpoint.getPose().getRotation().getDegrees() : () -> 0.0;
        BooleanSupplier inIntake = () -> currentlyInState(FSMStates.TOP_INTAKE, FSMStates.TOP_INTAKE_READY, FSMStates.GROUND_INTAKE, FSMStates.GROUND_INTAKE_READY);
        DoubleSupplier xyGain = () -> inIntake.getAsBoolean() ? 0.7 : 1;
        DoubleSupplier tGain = () -> inIntake.getAsBoolean() ? 0.3 : 1;
        Function<Double, Double> normalStickCurve = Util::halfLinearHalfCubic;
        Function<Double, Double> intakingStickCurve = (x) -> x;//Math.signum(x) * Math.sqrt(Math.abs(x));
        CommandScheduler.getInstance().setDefaultCommand(mecanum, new DefaultDriveCommand(
                mecanum,
                () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftY() / driverPad.getLeftX()) < 0.05 ? 0 : driverPad.getLeftY()) * xyGain.getAsDouble(),
                () -> Util.halfLinearHalfCubic(Math.abs(driverPad.getLeftX() / driverPad.getLeftY()) < 0.05 ? 0 : driverPad.getLeftX()) * xyGain.getAsDouble(),
                () -> (inIntake.getAsBoolean() ? intakingStickCurve : normalStickCurve).apply(driverPad.getRightX()) * tGain.getAsDouble(),
                fieldCentricHeading
        ));


        configureDriver();
        configureOperator();
        configureDualControl();


        waitForStart();
        // temporary pls remove later
        pinpoint.setPosition(-65, 40);
        while (!isStopRequested()) {
            telemetry.addData("current state", getState().toString());
            telemetry.addData("current intake state", intakeState.toString());
            telemetry.addData("motorpos", extension.getCurrentInches());
            telemetry.addData("pivotpos", pivot.getCurrentPosition());
            telemetry.addData("time", timer.milliseconds());
            telemetry.addData("hi", CommandScheduler.getInstance().getDefaultCommand(mecanum));
            telemetry.addData("intake flipped?", reverseClaw.get());
            //telemetry.addData("intake front voltage", intake.getFrontV());
            telemetry.addData("pose x", pinpoint.getPose().getX());
            telemetry.addData("pose y", pinpoint.getPose().getY());
            telemetry.addData("pose heading", pinpoint.getPose().getRotation().getDegrees());
            telemetry.addData("forward comp", mecanum.getForwardCompensationSupplier().getAsDouble());

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
                new ParallelCommandGroup(
                        new InterruptCommand(
                                new ExtendCommand(extension, SlideConstants.minExtension),
                                () -> extension.getCurrentInches() < 10.0
                        ),
                        new FullIntakeFoldCommand(intake, turret, wrist)).andThen(bucketPos()),
                bucketPos(),
                inState(FSMStates.TOP_INTAKE, FSMStates.TOP_INTAKE_READY, FSMStates.GROUND_INTAKE, FSMStates.GROUND_INTAKE_READY)
        ));

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(retract());

        // ------- INTAKE -------
        driverPad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                topIntakeReady(() -> 0),
                groundIntakeReady(),
                () -> intakeState == IntakeState.TOP
        )).whenReleased(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.GROUND) {
                        cs.schedule(groundIntake());
                    }
                })
        );

        driverPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                topIntakeReady(() -> 90),
                groundIntakeReady(),
                () -> intakeState == IntakeState.TOP
        )).whenReleased(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.GROUND) {
                        cs.schedule(groundIntake());
                    }
                })
        );

        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(new InstantCommand(() -> intakeState = IntakeState.TOP));
        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(new InstantCommand(() -> intakeState = IntakeState.GROUND));

        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new ConditionalCommand(
                topIntake(),
                new IntakeClawCommand(intake, IntakeConstants.openPos),
                () -> intakeState == IntakeState.TOP
        )).whenReleased(new ConditionalCommand(
                topIntakeReady(() -> lastTurretAngle),
                new IntakeClawCommand(intake, IntakeConstants.closedPos).andThen(new WaitCommand(200), retract()),
                () -> intakeState == IntakeState.TOP
        ));

        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ConditionalCommand(
                        new IntakeControlCommand(intake, IntakeConstants.openPos, -0.1),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        inState(FSMStates.OUTTAKE)
                )).whenReleased(new ConditionalCommand(
                        new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                        new IntakeClawCommand(intake, IntakeConstants.closedPos),
                        inState(FSMStates.TOP_INTAKE, FSMStates.GROUND_INTAKE)
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
