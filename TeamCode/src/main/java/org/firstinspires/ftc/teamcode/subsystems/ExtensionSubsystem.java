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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

import java.util.function.DoubleSupplier;

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

    //Triggers
    public Trigger underZero;
    public Trigger gainScheduleTrigger;
    public Trigger downwardsStallTrigger;
    public Trigger submersibleLimitTrigger;
    public Trigger maxExtensionLimitTrigger;
    public Trigger forwardTarget;
    public Trigger manualControlTrigger;


    public ExtensionSubsystem(HardwareMap hMap, PivotSubsystem pivotSubsystem, CachingVoltageSensor voltage) {
        this.voltage = voltage;
        this.pivotSubsystem = pivotSubsystem;

        motor0 = (DcMotorEx) hMap.dcMotor.get("slide0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1 = (DcMotorEx) hMap.dcMotor.get("slide1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor0.setCurrentAlert(4, CurrentUnit.AMPS);
        motor1.setCurrentAlert(4, CurrentUnit.AMPS);

        squid.setPID(SlideConstants.kP);

        this.initialize();
    }


    public void initialize() {
        manualControlTrigger = new Trigger(() -> manualControl);
        forwardTarget = new Trigger(() -> forwardTarget());
        underZero = new Trigger(() -> getCurrentPosition() < 0);
        gainScheduleTrigger = new Trigger(() -> getCurrentInches() > SlideConstants.bucketPosGainSchedulePos);
        downwardsStallTrigger = new Trigger(() -> getCurrentPosition() < 5)
                .and(new Trigger(() -> backwardPower()));
        submersibleLimitTrigger = new Trigger(() -> manualControl)
                        .and(new Trigger(() -> getCurrentInches() > SlideConstants.submersibleIntakeMaxExtension))
                        .and(forwardTarget)
                        .and(new Trigger(() -> pivotSubsystem.isClose(PivotConstants.intakePos)))
                        .whenActive(() -> Log.i("A", "Extension limit has been breached"));
        maxExtensionLimitTrigger = new Trigger(() -> getCurrentInches() >= SlideConstants.maxExtension)
                .and(forwardTarget)
                .whenActive(() -> Log.i("A", "Extension limit has been breached"));

        underZero.whenActive(this::reset);
        gainScheduleTrigger.whenActive(() -> squid.setPID(SlideConstants.kP * SlideConstants.bucketPosGainScheduleMult)).whenInactive(() -> squid.setPID(SlideConstants.kP));
        // Stall Detection is cooked because u might as well just have the driver run bucket or something to make sure it's unjammed
        // V good for award bait-
//        downwardsStallTrigger.whenActive(() -> resetOffset = getCurrentPosition());
//        submersibleLimitTrigger.whileActiveContinuous(stopC());
    }

    public boolean forwardTarget(){
        return targetInches - getCurrentInches() > 0;
    }

    public boolean backwardTarget(){
        return !forwardTarget();
    }

    public boolean forwardPower() {
        return motor0.getPower() > 0 && motor1.getPower() < 0;
    }

    public boolean backwardPower() {
        return !forwardPower();
    }

    public DoubleSupplier getVoltageScalarSupplier() {
        return voltage::getVoltageNormalized;
    }

    /**
     * @return A scalar that normalizes power outputs to the nominal voltage from the current voltage.
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

    public void setPowerMul(double extensionPowerMul) {
        this.extensionPowerMul = extensionPowerMul;
    }

    /**
     * Sets the power of the slides
     */
    public void openloop(double power) {
        motor0.setPower(power * SlideConstants.direction);
        motor1.setPower(-power * SlideConstants.direction);
    }

    public Command enableManualControl(){
        return new InstantCommand(() -> this.manualControl = true);
    }

    public Command disableManualControl(){
        return new InstantCommand(() -> this.manualControl = false);
    }

    public void setManualControl(boolean set) {
        this.manualControl = set;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    public void setTargetInches(double inches){
        targetInches = inches;
    }

    /**
     * this sets the target as well
     */
    private void extendInches(double inches) {
        targetInches = inches;
        extendToPosition((int) (inches * SlideConstants.ticksPerInch));
    }

    private void extendToPosition(int ticks) {
        // extensionPowerMul only applies to the squid output because the feedforward should stay constant
        double power =
                +squid.calculate(ticks, getCurrentPosition()) * extensionPowerMul
                        + SlideConstants.FEEDFORWARD_top * Math.sin(Math.toRadians(pivotSubsystem.getCurrentPosition()));

        power *= voltage.getVoltageNormalized();

        openloop(power);
    }

    /**
     * @param target in inches, use the same one as the pid target
     */
    public boolean isClose(double target) {
        return Util.inRange(target, getCurrentInches(), SlideConstants.tolerance);
    }

    /**
     * @return position in ticks
     */
    public double getCurrentPosition() {
        return currentPos;
    }

    public double getCurrentInches() {
        return getCurrentPosition() / SlideConstants.ticksPerInch;
    }

    //Tempted to make this private lowk so also cancels everything else in the command scheduler, cuz ur forced to use stopC
    public void stop() {
        openloop(0);
    }

    public Command stopC() {
        return new RunCommand(() -> stop(), this);
    }

    public long getReasonableExtensionMillis(double targetInches) {
        return (long) (Math.abs(targetInches - getCurrentInches()) * SlideConstants.millisPerInch);
    }

    public void reset() {
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetOffset = 0;
    }

    @Override
    public void periodic() {
        //Hardware Access every loop
        currentPos = -motor0.getCurrentPosition() - resetOffset;

        if (manualControlTrigger.negate().and(maxExtensionLimitTrigger.negate()).and(submersibleLimitTrigger.negate()).get()){
            extendInches(targetInches);
        }

        FtcDashboard.getInstance().getTelemetry().addData("slide position", this.getCurrentInches());
        FtcDashboard.getInstance().getTelemetry().addData("slide motor power", motor0.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("manualControl", manualControlTrigger.get());
        FtcDashboard.getInstance().getTelemetry().addData("maxExtension", maxExtensionLimitTrigger.get());

    }
}

