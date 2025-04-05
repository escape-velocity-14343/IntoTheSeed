package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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

import org.firstinspires.ftc.teamcode.commands.custom.BucketAlignCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.LowBucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;

public abstract class Robot extends LinearOpMode {

    public enum FSMStates {
        READY,
        INTAKE_READY,
        INTAKE,
        HANG,
        OUTTAKE,
        SPECIMEN,
        FOLD,
        BASKET_ALIGN,
    }
    public enum StateProgress {
        NONE,
        PROGRESS,
        READY
    }

    public FSMStates robotState = FSMStates.READY;
    public StateProgress robotProgress = StateProgress.NONE;
    public AtomicBoolean reverseClaw = new AtomicBoolean(false);
    public AtomicBoolean lowBucket = new AtomicBoolean(false);

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
    public PtoSubsystem PTO;
    public VisionSubsystem vision;
    public TargetingSubsystem target;


    public IMU imu;

    public ElapsedTime timer = new ElapsedTime();
    public CommandScheduler cs = CommandScheduler.getInstance();

    protected double lastTurretAngle;

    public void initialize() {
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
        extension = new ExtensionSubsystem(hardwareMap, pivot, voltage);
        wrist = new WristSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        PTO = new PtoSubsystem(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        pivot.setExtensionSupplier(extension::getCurrentInches);
        PTO.setEngaged(false);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        target = new TargetingSubsystem(vision, pinpoint, telemetry);

        cs.registerSubsystem(basketSensor, pinpoint, mecanum, pivot, extension, wrist, intake, turret, PTO, vision, target);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    public void end() {
        cs.reset();
    }

    public Command intakePos() {
        return new IntakePosCommand(extension, pivot, wrist, intake)
                .andThen(setStateCommand(FSMStates.INTAKE));
    }

    public Command intake() {
        return new SubPosCommand(extension, wrist, intake, pivot, SlideConstants.submersibleIntakeMaxExtension)
                .andThen(setStateCommand(FSMStates.INTAKE));
    }

    public Command intake(DoubleSupplier forwardInches) {
        return new SubPosCommand(extension, wrist, intake, pivot, forwardInches)
                .andThen(setStateCommand(FSMStates.INTAKE));
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

    public Command intakeReady(DoubleSupplier turretAngle, DoubleSupplier forwardInches) {
        return new SubPosReadyCommand(
                extension,
                pivot,
                wrist,
                intake,
                turret,
                turretAngle,
                forwardInches,
                notInAnyState(FSMStates.INTAKE_READY, FSMStates.INTAKE)
        ).alongWith(new InstantCommand(() -> lastTurretAngle = turretAngle.getAsDouble())).andThen(setStateCommand(FSMStates.INTAKE_READY));
    }

    public Command intakeReady(DoubleSupplier turretAngle) {
        return new SubPosReadyCommand(
                extension,
                pivot,
                wrist,
                intake,
                turret,
                turretAngle,
                SlideConstants.submersibleIntakeMaxExtension,
                notInAnyState(FSMStates.INTAKE_READY, FSMStates.INTAKE)
        ).alongWith(new InstantCommand(() -> lastTurretAngle = turretAngle.getAsDouble())).andThen(setStateCommand(FSMStates.INTAKE_READY));
    }

    public Command intakeReady() {
        return intakeReady(() -> lastTurretAngle);
    }

    public Command bucketAlign() {
        return new BucketAlignCommand(mecanum, basketSensor, pinpoint).whenClose(48.0, bucketPos()).alongWith(setStateCommand(FSMStates.BASKET_ALIGN));
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
