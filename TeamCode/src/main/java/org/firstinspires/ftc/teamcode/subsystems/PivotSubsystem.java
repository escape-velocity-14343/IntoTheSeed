package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.AnalogEncoder;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

public class PivotSubsystem extends SubsystemBase {
    private DcMotor motor0, motor1;
    private double currentPos = 0;
    private double pivotVelocity = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private boolean manualControl = false;
    private PIDController pid = new PIDController(PivotConstants.kPRetracted, 0, PivotConstants.kD);
    AnalogEncoder encoder;
    private CachingVoltageSensor voltage;
    private DoubleSupplier extensionInches = () -> 0;
    private boolean supplierSet = false;
    private Trigger extensionSetTrigger = new Trigger(() -> !supplierSet).whileActiveContinuous(() -> Log.i("WARNING", "PIVOT EXTENSION SUPPPLIER UNSET"));
    public Trigger manualControlTrigger = new Trigger(() -> manualControl);

    public boolean useKg = true;

    public PivotSubsystem(HardwareMap hMap, CachingVoltageSensor voltage) {
        motor0 = hMap.dcMotor.get("tilt0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1 = hMap.dcMotor.get("tilt1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encoder = new AnalogEncoder("sensOrange", hMap);
        encoder.setPositionOffset(PivotConstants.encoderOffset);
        encoder.setInverted(PivotConstants.encoderInvert);

        this.voltage = voltage;

        timer.reset();
    }

    public void setExtensionSupplier(DoubleSupplier extensionInches) {
        this.extensionInches = extensionInches;
        supplierSet = true;
    }

    public Command openloopC(DoubleSupplier power){
        return new RunCommand(() -> openloopS(power), this);
    }

    /**
     * internal factory method for openloop power
     * @param power
     */
    private void openloopS(DoubleSupplier power){
        openloop(power.getAsDouble());
    }

    public void openloop(double power) {
        // Log.v("pivot", "power: " + power);
        motor0.setPower(power * PivotConstants.direction);
        motor1.setPower(-power * PivotConstants.direction);
    }

    public void tiltToPos(double target) {
        manualControl = false;
        setTarget(target);
        double power =
                pid.calculate(target, getCurrentPosition()) + getKg();

        if (isNear(target, 0.5)){
            power = 0;
        }
        // if (currentPos > PivotConstants.topLimit-1 && power >= 0) {
        //    power = 0.3;
        // }
        if (power <= 0 && isClose(target) && target == PivotConstants.bottomLimit) {
            power = -0.05;
        }
        /*if (power > 0 && currentPos < 20) {
            power *= PivotConstants.bottomPMult;
        }*/

        if (power < 0) {
            power = -Math.min(Math.abs(power), 1 - (getKg() * 2));
        }

        //stop breaking belt, doesnt seem to cause problems
        if (power < 0 && currentPos < PivotConstants.powerCutAngle && useKg) {
            power = 0;
        }

        power = Range.clip(power, -1.0, 1.0);

        power *= voltage.getVoltageNormalized();

        openloop(power);
    }

    public boolean isNear(double target, double tolerance){
        return Math.abs(target-getCurrentPosition()) < tolerance;
    }

    public Command getPivotCommand(double target){
        return new PivotCommand(this, target);
    }

    public Command getPivotCommand(DoubleSupplier target){
        return new RunCommand(() -> setTarget(target.getAsDouble()), this);
    }

    public Command getPivotCommandInstant(DoubleSupplier target){
        return new InstantCommand(() -> setTarget(target.getAsDouble()), this);
    }

    public void setTarget(DoubleSupplier target){
        manualControl = false;

    }

    public void setTarget(double target) {
        manualControl = false;
        this.target = target;
    }

    public double getPivotVelocity() {
        return pivotVelocity;
    }

    /**
     * @param target in inches, use the same one as the pid target
     */
    public boolean isClose(double target) {
        return isClose(target, PivotConstants.tolerance); // || currentPos < PivotConstants.bottomLimit;
    }

    /**
     * @param target in inches, use the same one as the pid target
     */
    public boolean isClose(double target, double tolerance) {
        return Util.inRange(target, currentPos, tolerance); // || currentPos < PivotConstants.bottomLimit;
    }
    public boolean isDone() {
        return isClose(target);
    }

    /**
     * @return In degrees
     */
    public double getCurrentPosition() {
        return currentPos;
    }

    public Command enableManualControl() {
        return new InstantCommand(() -> this.manualControl = true);
    }

    public Command disableManualControl() {
        return new InstantCommand(() -> this.manualControl = false);
    }

    public void setManualControl(boolean manualControl) {
        this.manualControl = manualControl;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    public void stop() {
        motor0.setPower(0);
        motor1.setPower(0);
    }

    private double interpolateKp(double x) {
        double x1 = 0;
        double y1 = PivotConstants.kPRetracted;
        double x2 = SlideConstants.bucketPos;
        double y2 = PivotConstants.kPExtended;

        return y1 + x * (y2 - y1) / (x2 - x1);
    }

    private double interpolateKg(double x) {
        double x1 = 0;
        double y1 = PivotConstants.kGRetracted;
        double x2 = SlideConstants.bucketPos;
        double y2 = PivotConstants.kGFullyExtended;

        return y1 + x * (y2 - y1) / (x2 - x1);
    }

    private double interpolatedRawFeedforward(){
        return interpolateKp(extensionInches.getAsDouble());
    }

    private double interpolatedRawFeedforwardkG(){
        return interpolateKg(extensionInches.getAsDouble());
    }

    private double getKg(){
        return useKg ? (interpolatedRawFeedforwardkG() * Math.cos(Math.toRadians(getCurrentPosition()))) : 0;
    }

    @Override
    public void periodic() {

        pid.setP(PivotConstants.kPRetracted
                + (extensionInches.getAsDouble() / SlideConstants.maxExtension)
                * (PivotConstants.kPExtended - PivotConstants.kPRetracted));
        //Cache last position
        double lastPos = currentPos;
        pivotVelocity = (lastPos - currentPos) / timer.seconds();

        //Update encoder reading every loop
        currentPos = encoder.getAngle();
        pivotVelocity = (lastPos - currentPos) / timer.seconds();
        if (!manualControl) {
            tiltToPos(target);
        }
        //Timer reset
        timer.reset();
    }
    public void setUseKg(boolean use) {
        useKg = use;
    }
}
