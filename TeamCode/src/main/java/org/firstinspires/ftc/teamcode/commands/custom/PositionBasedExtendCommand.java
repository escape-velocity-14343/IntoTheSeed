package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import javax.annotation.Nullable;

public class PositionBasedExtendCommand extends CommandBase {

    ExtensionSubsystem extend;
    double target;

    private DoubleSupplier targetSupplier;
    @Nullable
    private Double powerMul = null;
    private double oldExtensionPowerMul;
    private boolean isEnding = true;

    /**
     * @param subsystem
     * @param target    in inches
     */
    public PositionBasedExtendCommand(ExtensionSubsystem subsystem, double target) {
        this.extend = subsystem;
        this.target = target;
        addRequirements(subsystem);
    }

    public PositionBasedExtendCommand(ExtensionSubsystem subsystem, DoubleSupplier target) {
        this.extend = subsystem;
        this.targetSupplier = target;
        addRequirements(subsystem);
    }

    /**
     * @param subsystem
     * @param target    in inches
     * @param powerMul  the power multipler to feed into the subsystem, will reset to the previous
     *                  power multiplier once command finishes
     */
    public PositionBasedExtendCommand(ExtensionSubsystem subsystem, double target, double powerMul) {
        this(subsystem, target);
        this.powerMul = powerMul;
    }

    @Override
    public void initialize() {
        if (powerMul != null) {
            oldExtensionPowerMul = extend.getPowerMul();
            extend.setPowerMul(powerMul);
        }
        if (Objects.nonNull(targetSupplier)) {
            this.target = targetSupplier.getAsDouble();
        }
        extend.setTargetInches(target);
        extend.setManualControl(false);
    }

    @Override
    public boolean isFinished() {
        return isEnding && extend.isClose(target);
    }

    @Override
    public void end(boolean wasInterrupted) {
        if (powerMul != null) {
            extend.setPowerMul(oldExtensionPowerMul);
        }
        Log.i("%9", "Extension to " + target);
    }

    public boolean isEnding() {
        return isEnding;
    }

    public PositionBasedExtendCommand setEnding(boolean ending) {
        this.isEnding = ending;
        return this;
    }
}
