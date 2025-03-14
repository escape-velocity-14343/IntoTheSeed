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
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.LowBucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;

public abstract class Robot extends LinearOpMode {

    public enum FSMStates {
        INTAKE,
        HANG,
        OUTTAKE,
        SPECIMEN,
        FOLD,
        AUTOSCORE,
        NONE
    }

    public FSMStates robotState = FSMStates.NONE;
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
    public CachingVoltageSensor voltage;
    public BasketSensorSubsystem basketSensor;
    public TurretSubsystem turret;

    public IMU imu;
    // public CameraSubsystem cam;
    public ElapsedTime timer = new ElapsedTime();
    public CommandScheduler cs = CommandScheduler.getInstance();

    public void initialize() {
        DriveConstants.highExtend = false;
        AutoConstants.subBarrierY = 24.0;
        // cs.reset();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        // otos = new OTOSSubsystem(hardwareMap);
        voltage = new CachingVoltageSensor(hardwareMap);
        // basketSensor = new BasketSensorSubsystem(hardwareMap);

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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        pivot.setExtensionSupplier(extension::getCurrentInches);

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
                .alongWith(new InstantCommand(() -> setState(FSMStates.INTAKE)));
    }

    public Command subPos() {
        return new SubPosCommand(extension, wrist, intake, pivot)
                .alongWith(new InstantCommand(() -> setState(FSMStates.INTAKE)));
    }

    public Command bucketPos() {
        return new ConditionalCommand(
                LowBucketPosCommand.newWithWristPos(extension, pivot, wrist),
                new BucketPosCommand(extension, pivot, wrist, turret),
                lowBucket::get);
    }

    public void setState(FSMStates state) {
        robotState = state;
    }

    public FSMStates getState() {
        return robotState;
    }
}
