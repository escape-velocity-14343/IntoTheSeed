package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

public class ExtensionSubsystem extends SubsystemBase {
    private final DcMotorEx motor0;
    private final DcMotorEx motor1;
    private final CachingVoltageSensor voltage;
    private int currentPos = 0;
    private double targetInches = 0;
    private final SquIDController squid = new SquIDController();
    private boolean manualControl = false;
    private int resetOffset = 0;
    private double extensionPowerMul = 1.0;

    private final PivotSubsystem pivotSubsystem;

    // Triggers
    public Trigger underZeroTrigger;
    public Trigger gainScheduleTrigger;
    public Trigger downwardsStallTrigger;
    public Trigger submersibleLimitTrigger;
    public Trigger maxExtensionLimitTrigger;
    public Trigger forwardTargetTrigger;
    public Trigger manualControlTrigger;
    public Trigger extendedTrigger;

    public ExtensionSubsystem(
            HardwareMap hMap, PivotSubsystem pivotSubsystem, CachingVoltageSensor voltage) {
        this.voltage = voltage;
        this.pivotSubsystem = pivotSubsystem;

        motor0 = (DcMotorEx) hMap.dcMotor.get("slide0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1 = (DcMotorEx) hMap.dcMotor.get("slide1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor0.setCurrentAlert(SlideConstants.alertCurrent, CurrentUnit.AMPS);
        motor1.setCurrentAlert(SlideConstants.alertCurrent, CurrentUnit.AMPS);

        squid.setPID(SlideConstants.kP);

        this.initialize();
    }

    /**
     * Conditions for Extension losing power: 1. manualControl is true 2. past extension limits, GDC
     * and physical limitations
     */
    private void initialize() {
        manualControlTrigger = new Trigger(() -> manualControl);
        forwardTargetTrigger = new Trigger(() -> forwardTarget());
        underZeroTrigger = new Trigger(() -> getCurrentPosition() < 0);
        downwardsStallTrigger =
                new Trigger(() -> getCurrentPosition() < 5).and(new Trigger(() -> backwardPower()));
        submersibleLimitTrigger =
                new Trigger(() -> manualControl)
                        .and(
                                new Trigger(
                                        () ->
                                                getCurrentInches()
                                                        > SlideConstants
                                                                .submersibleIntakeMaxExtension))
                        .and(forwardTargetTrigger)
                        .and(new Trigger(() -> pivotSubsystem.isClose(PivotConstants.intakePos)))
                        .whenActive(
                                () -> Log.i("A", "Submersible Extension limit has been breached"));
        maxExtensionLimitTrigger =
                new Trigger(() -> getCurrentInches() >= SlideConstants.maxExtension)
                        .and(forwardTargetTrigger)
                        .whenActive(() -> Log.i("A", "Maximum Extension limit has been breached"));
        extendedTrigger = new Trigger(() -> getCurrentInches() > SlideConstants.extendedThreshold);

        underZeroTrigger.whenActive(this::reset);
        // Stall Detection is cooked because u might as well just have the driver run bucket or
        // something to make sure it's unjammed
        // V good for award bait-
        //        downwardsStallTrigger.whenActive(() -> resetOffset = getCurrentPosition());
        //        submersibleLimitTrigger.whileActiveContinuous(stopC());
    }

    /**
     * Checks whether the intended direction is up or down. <br>
     * Is useful for extension limits, since AND conditions mean the power will never be set and
     * cannot be observed.
     *
     * @return boolean
     */
    public boolean forwardTarget() {
        return targetInches - getCurrentInches() > 0;
    }

    /**
     * Does the opposite of @forwardTarget
     *
     * @return boolean
     */
    public boolean backwardTarget() {
        return !forwardTarget();
    }

    /**
     * Checks direction of applied power.
     *
     * @return boolean
     */
    public boolean forwardPower() {
        return motor0.getPower() > 0 && motor1.getPower() < 0;
    }

    /**
     * Does the opposite of @forwardPower
     *
     * @return boolean
     */
    public boolean backwardPower() {
        return !forwardPower();
    }

    public DoubleSupplier getVoltageScalarSupplier() {
        return voltage::getVoltageNormalized;
    }

    /**
     * A voltage scalar that normalizes power outputs to the nominal voltage from the current
     * voltage
     *
     * @return double
     */
    public double getVoltageScalar() {
        return voltage.getVoltageNormalized();
    }

    public DoubleSupplier getPowerMulSupplier() {
        return this::getPowerMul;
    }

    public double getPowerMul() {
        return extensionPowerMul;
    }

    /**
     * Sets scalar for slide extension speed, ideally for manual control scaling
     *
     * @param extensionPowerMul
     */
    public void setPowerMul(double extensionPowerMul) {
        this.extensionPowerMul = extensionPowerMul;
    }

    /**
     * Factory method for use inside a sequential command group
     *
     * @param extensionPowerMul
     * @return Command
     */
    public Command setPowerMulC(double extensionPowerMul) {
        return new InstantCommand(() -> setPowerMul(extensionPowerMul));
    }

    /**
     * Sets openloop power to the motors.
     * <br> <br>
     * Returns a InstantCommand factory, for setting the motor powers instantaneously.
     * @param power
     * @return
     */
    public Command openloopC(Double power){
        if (manualControl){
            return new InstantCommand(() -> openloop(power), this);
        }
        Log.i("WARNING", "RAN OPEN LOOP WHEN MANUAL CONTROL WAS NOT ENABLED");
        return new InstantCommand();
    }

    /**
     * Sets openloop power to the motors.
     * <br><br>
     * Returns a RunCommand factory, for binding to operator triggers during manual control,
     * and for pure open loop control, like for stalling the robot.
     * <br> Bind to a Trigger that is decorated with the manual control Trigger!
     * <br> Throws warning if manual control is not enabled.
     * @param power
     * @return RunCommand Factory
     */
    public Command openloopC(DoubleSupplier power){
        if (manualControl){
            return new RunCommand(() -> openloopS(power), this);
        }
        Log.i("WARNING", "RAN OPEN LOOP WHEN MANUAL CONTROL WAS NOT ENABLED");
        return new InstantCommand();
    }

    /**
     * Internal factory method for openloop slide control
     * @param power
     * @return
     */
    private void openloopS(DoubleSupplier power){
        openloop(power.getAsDouble());
    }

    /**
     * Open Loop Slide Control
     *
     * @param power from [-1.0 to 1.0]
     */
    private void openloop(double power) {
        motor0.setPower(power * SlideConstants.direction);
        motor1.setPower(-power * SlideConstants.direction);
    }

    public Command enableManualControl() {
        return new InstantCommand(() -> this.manualControl = true);
    }

    public Command disableManualControl() {
        return new InstantCommand(() -> this.manualControl = false);
    }

    public void setManualControl(boolean set) {
        this.manualControl = set;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    /**
     * Intended Access/Entry Point for ExtensionSubsystem
     * <br> The way it's wrapped with Extend command adds logging and end state functionality.
     * @param inches
     */
    public void setTargetInches(double inches) {
        targetInches = inches;
    }

    /**
     * Regular Extend Command reference owned in the subsystem. <br>
     * See @ExtendCommand for actual implementation.
     * @param target
     * @return Command
     */
    public Command getExtendCommand(double target){
        return new ExtendCommand(this, target);
    }

    /**
     * Run Command factory for extend command, that never ends.
     * @param target
     * @return Command
     */
    public Command getExtendCommand(DoubleSupplier target){
        return new RunCommand(() -> this.setTargetInches(target.getAsDouble()), this);
    }

    /**
     * Internal Factory Method
     *
     * @param inches
     */
    private void extendInches(double inches) {
        targetInches = inches;
        extendToPosition((int) (inches * SlideConstants.ticksPerInch));
    }

    /**
     * Internal Factory Method
     *
     * @param ticks
     */
    private void extendToPosition(int ticks) {
        // extensionPowerMul only applies to the squid output because the feedforward should stay
        // constant
        double power =
                +squid.calculate(ticks, getCurrentPosition()) * extensionPowerMul
                        + SlideConstants.kS
                        + interpolate(getCurrentInches())
                                * Math.sin(Math.toRadians(pivotSubsystem.getCurrentPosition()));

        power *= voltage.getVoltageNormalized();

        openloop(power);
    }

    private double interpolate(double x) {
        double x1 = 0;
        double y1 = SlideConstants.FEEDFORWARD_bottom;
        double x2 = SlideConstants.bucketPos;
        double y2 = SlideConstants.FEEDFORWARD_top;

        return y1 + (x) * (y2 - y1) / (x2 - x1);
    }

    /**
     * @param target in inches, uses the same one as the pid target
     */
    public boolean isClose(double target) {
        return Util.inRange(target, getCurrentInches(), SlideConstants.tolerance);
    }

    /**
     * Get current position in ticks
     *
     * @return double
     */
    public double getCurrentPosition() {
        return currentPos;
    }

    /**
     * Gets current position in Inches
     *
     * @return double
     */
    public double getCurrentInches() {
        return getCurrentPosition() / SlideConstants.ticksPerInch;
    }

    /** Would be useful as an e-stop, binded to some trigger. */
    public void stop() {
        openloop(0);
    }

    /**
     * Factory for stop()
     *
     * @return Command
     */
    public Command stopC() {
        return new RunCommand(() -> stop(), this);
    }

    /**
     * Gets upper bound on how long it should take us to go from current pos to target pos, in ms.
     * <br>
     * Used for timeouts.
     *
     * @param targetInches
     * @return long
     */
    public long getReasonableExtensionMillis(double targetInches) {
        return (long) (Math.abs(targetInches - getCurrentInches()) * SlideConstants.millisPerInch);
    }

    /** Resets the position of the slides */
    public void reset() {
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetOffset = 0;
    }

    /**
     * Factory command for reseting the position of the slides
     */
    public Command resetC(){
        return new InstantCommand(this::reset, this);
    }

    /** The periodic loop for the subsystem. */
    @Override
    public void periodic() {
        // Hardware Access every loop
        currentPos = -motor0.getCurrentPosition() - resetOffset;

        if (manualControlTrigger
                .negate()
                .and(maxExtensionLimitTrigger.negate())
                .and(submersibleLimitTrigger.negate())
                .get()) {
            extendInches(targetInches);
        }

        FtcDashboard.getInstance()
                .getTelemetry()
                .addData("slide position", this.getCurrentInches());
        FtcDashboard.getInstance().getTelemetry().addData("slide motor power", motor0.getPower());
        FtcDashboard.getInstance()
                .getTelemetry()
                .addData("manualControl", manualControlTrigger.get());
        FtcDashboard.getInstance()
                .getTelemetry()
                .addData("maxExtension", maxExtensionLimitTrigger.get());
    }
}
