package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.commands.custom.BucketRelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtensionPowerCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.AutoSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGVFCommand;
import org.firstinspires.ftc.teamcode.commands.group.GroundSubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.GroundSubReadyPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.LowBucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.path.spline.CubicBezier;
import org.firstinspires.ftc.teamcode.lib.path.spline.Spline;

public abstract class Robot extends LinearOpMode {

    public enum FSMStates {
        READY,
        TOP_INTAKE_READY,
        TOP_INTAKE,
        GROUND_INTAKE_READY,
        GROUND_INTAKE,
        HANG_READY,
        HANG_L2,
        HANG_L3,
        OUTTAKE,
        SPECIMEN,
        FOLD,
        BUCKET_ALIGN,
    }

    public enum StateProgress {
        NONE,
        PROGRESS,
        READY
    }

    public FSMStates robotState = FSMStates.READY;
    public StateProgress robotProgress = StateProgress.NONE;
    public AtomicBoolean reverseClaw = new AtomicBoolean(false);
    public static AtomicBoolean lowBucket = new AtomicBoolean(false);

    public List<LynxModule> hubs;
    public ExtensionSubsystem extension;
    public MecanumDriveSubsystem mecanum;
    public PivotSubsystem pivot;
    public WristSubsystem wrist;
    public IntakeSubsystem intake;
    // @Deprecated
    // public OTOSSubsystem otos;
    public PinpointSubsystem pinpoint;
    //    public VisionSubsystem visionSubsystem;
    public CachingVoltageSensor voltage;
    public BucketSensorSubsystem basketSensor;
    public TurretSubsystem turret;
    public PtoSubsystem pto;
    public VisionSubsystem vision;
    public TargetingSubsystem target;


    public IMU imu;

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime loopTime = new ElapsedTime();
    private int cycles = 0;
    public CommandScheduler cs = CommandScheduler.getInstance();

    protected double lastTurretAngle;

    public void initialize() {
        loopTime.reset();
        SlideConstants.highExtend = false;

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltage = new CachingVoltageSensor(hardwareMap);
        basketSensor = new BucketSensorSubsystem(hardwareMap);

        pinpoint = new PinpointSubsystem(hardwareMap);

        mecanum =
                new MecanumDriveSubsystem(
                        "frontRight",
                        "frontLeft",
                        "backRight",
                        "backLeft",
                        hardwareMap,
                        pinpoint,
                        voltage);
        pivot = new PivotSubsystem(hardwareMap, voltage);
        pto = new PtoSubsystem(hardwareMap);
        extension = new ExtensionSubsystem(hardwareMap, pivot, voltage, mecanum, pto);
        wrist = new WristSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        pivot.setExtensionSupplier(extension::getCurrentInches);
        pto.setEngaged(false);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        target = new TargetingSubsystem(vision, pinpoint, telemetry);

        mecanum.setForwardCompensationSupplier(() -> extension.getCurrentInches() / SlideConstants.maxExtension * DriveConstants.forwardMotorMultiplier * Math.cos(Math.toRadians(pivot.getCurrentPosition())) + 1.0);

        cs.registerSubsystem(basketSensor, pinpoint, mecanum, pivot, extension, wrist, intake, turret, pto, vision, target);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        CommandScheduler.getInstance().run();

        if (cycles == 0) {
            loopTime.reset();
        }

        cycles++;
        telemetry.addData("HZ", "" + cycles / loopTime.seconds());

        telemetry.update();
    }

    public void end() {
        cs.reset();
    }

    public Command topIntake() {
        return new SubPosCommand(extension, wrist, intake, pivot, SlideConstants.submersibleIntakeMidExtension)
                .andThen(setStateCommand(FSMStates.TOP_INTAKE));
    }

    public Command bucketPos() {
        return new ConditionalCommand(
                LowBucketPosCommand.newWithWristPos(extension, pivot, wrist),
                new BucketPosCommand(extension, pivot, wrist, turret),
                lowBucket::get
        ).andThen(setStateCommand(FSMStates.OUTTAKE));
    }

    public Command retract() {
        return new RetractCommand(wrist, pivot, extension, turret, intake).andThen(setStateCommand(FSMStates.READY));
    }

    public Command topIntakeReady(DoubleSupplier turretAngle) {
        return new SubPosReadyCommand(
                extension,
                pivot,
                wrist,
                intake,
                turret,
                turretAngle,
                SlideConstants.submersibleIntakeMidExtension,
                notInAnyState(FSMStates.TOP_INTAKE_READY, FSMStates.TOP_INTAKE)
        ).alongWith(new InstantCommand(() -> lastTurretAngle = turretAngle.getAsDouble())).andThen(setStateCommand(FSMStates.TOP_INTAKE_READY));
    }

    public Command topIntakeReady() {
        return topIntakeReady(() -> lastTurretAngle);
    }

    public Command groundIntakeReady() {
        return new ConditionalCommand(
                retract(),
                new InstantCommand(),
                inState(FSMStates.OUTTAKE)
        ).andThen(new GroundSubReadyPosCommand(
                extension,
                pivot,
                intake,
                turret,
                SlideConstants.submersibleIntakeGroundMaxExtension
        )).andThen(setStateCommand(FSMStates.GROUND_INTAKE_READY));
    }

    public Command groundIntake() {
        return new ConditionalCommand(
                retract(),
                new InstantCommand(),
                inState(FSMStates.OUTTAKE)
        ).andThen(new GroundSubPosCommand(
                extension,
                pivot,
                wrist,
                intake,
                turret,
                SlideConstants.submersibleIntakeGroundMaxExtension
        )).andThen(setStateCommand(FSMStates.GROUND_INTAKE));
    }

    public Command bucketAlign() {
        //return new BucketAlignCommand(mecanum, basketSensor, pinpoint).whenClose(48.0, bucketPos()).alongWith(setStateCommand(FSMStates.BASKET_ALIGN));
        return new InstantCommand(() -> {
            double x = pinpoint.getPose().getX();
            double y = pinpoint.getPose().getY();
            Spline generatedSpline = new CubicBezier(
                    x, y,
                    -30 - 0.5 * x + 10, 35,
                    -52, 48,
                    //-62, 58 Arbitrary Innaias points from a week ago
                    AutoConstants.ultrasonicBucketPos.getX(), AutoConstants.ultrasonicBucketPos.getY()
            );
            if (currentlyInState(
                    FSMStates.TOP_INTAKE, FSMStates.TOP_INTAKE_READY,
                    FSMStates.GROUND_INTAKE, FSMStates.GROUND_INTAKE_READY
            )) {
                // sus ඞ
                cs.schedule(new InstantCommand(() -> {
                    extension.setTargetInches(0.0);
                    wrist.setWrist(IntakeConstants.foldedPos);
                }));
            }
            cs.schedule(
                    new DefaultGVFCommand(mecanum, pinpoint, generatedSpline)
                            .whenClose(bucketPos(), 48.0)
                            .whenClose(new WaitCommand(300).andThen(new BucketRelocalizeCommand(basketSensor, pinpoint, 3)), 1.0)
                            .setTangentOffset(180)
                            //.endWhenClose(1.0)
                            .alongWith(setStateCommand(FSMStates.BUCKET_ALIGN))
                            //.whenFinished(() -> cs.schedule(
                            //        new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-63, 59, Rotation2d.fromDegrees(-45)))
                            //                .alongWith(new WaitCommand(200).andThen(new BucketRelocalizeCommand(basketSensor, pinpoint, 3.0)))
                            //                .interruptOn(() -> Util.isGamepadAlive(gamepad1, 0.5))
                            //))
                            .interruptOn(() -> Util.isGamepadAlive(gamepad1, 0.5))
            );
        });
    }

    public Command hangL2Ready() {
        return new ParallelCommandGroup(
                new ExtendCommand(extension, SlideConstants.hangReady),
                new PivotCommand(pivot, PivotConstants.hangReady),
                new WristCommand(wrist, IntakeConstants.hangReady)
        ).andThen(setStateCommand(FSMStates.HANG_READY));
    }
    public Command hangL3Ready() {
        return new ParallelCommandGroup(
                new ExtendCommand(extension, SlideConstants.hangL3Ready),
                new PivotCommand(pivot, PivotConstants.hangL3Ready),
                new WristCommand(wrist, IntakeConstants.hangReady)
        ).andThen(setStateCommand(FSMStates.HANG_READY));
    }

    public Command hangL2() {
        return new SequentialCommandGroup(
                setStateCommand(FSMStates.HANG_L2),
                new InstantCommand(() -> pivot.setUseKg(false)),
                new InstantCommand(() -> pivot.setTarget(PivotConstants.hangPuyallup)),
                new InstantCommand(() -> wrist.setPwmDisabled(true)),
                new TimeoutCommand(new ExtendCommand(extension, 0.0), 1000),
                new ExtensionPowerCommand(extension, mecanum, pto, -1).alongWith(
                        new TimeoutCommand(new WaitUntilCommand(() -> extension.isClose(0.0, 1.0)), 3000).andThen(
                                new InstantCommand(() -> pivot.setUseKg(false)),
                                new PivotCommand(pivot, PivotConstants.hangIntermediate),
                                new InstantCommand(() -> pivot.setUseKg(true))
                        )
                ).withTimeout(10000)
        );
    }

    public void setState(FSMStates state) {
        robotState = state;
    }

    public Command setStateCommand(FSMStates state) {
        return new InstantCommand(() -> setState(state));
    }

    public FSMStates getState() {
        return robotState;
    }

    public BooleanSupplier inState(FSMStates... state) {
        return () -> Arrays.stream(state).anyMatch((testState) -> getState() == testState);
    }

    public boolean currentlyInState(FSMStates... state) {
        return Arrays.stream(state).anyMatch((testState) -> getState() == testState);
    }

    public BooleanSupplier notInState(FSMStates state) {
        return () -> getState() != state;
    }

    public BooleanSupplier notInAnyState(FSMStates... state) {
        return () -> Arrays.stream(state).allMatch((testState) -> getState() != testState);
    }
}
