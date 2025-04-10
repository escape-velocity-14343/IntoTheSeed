package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PtoSubsystem;

/**
 * Applies an open loop amount of power (and optionally through the PTO as well). Never finishes, so
 * it needs to be chained with something else.
 */
public class ExtensionPowerCommand extends CommandBase {
    private final ExtensionSubsystem extension;
    private final PtoSubsystem pto;
    private final double power;
    private boolean lastManualControl;

    public ExtensionPowerCommand(ExtensionSubsystem extension, double power) {
        this.extension = extension;
        this.pto = null;
        this.power = power;
        addRequirements(extension);
    }

    /**
     * Runs with the pto too
     * @param extension extension subsystem
     * @param mecanum mecanum subsystem (required to properly override the subsystem's default
     *                command, since the extension would otherwise interfere with the mecanum)
     * @param pto pto subsystem (required for pto)
     * @param power the power to use (negative retracts)
     */
    public ExtensionPowerCommand(ExtensionSubsystem extension, MecanumDriveSubsystem mecanum, PtoSubsystem pto, double power) {
        this.extension = extension;
        this.pto = pto;
        this.power = power;
        addRequirements(extension, mecanum, pto);
    }

    @Override
    public void initialize() {
        if (pto != null) {
            pto.setEngaged(true);
        }

        Log.i("extension power", "hi");
        lastManualControl = extension.getManualControl();
        extension.setManualControl(true);
    }

    @Override
    public void execute() {
        Log.i("extension power set", "open looping");
        extension.openloop(power);
    }

    @Override
    public void end(boolean interrupted) {
        extension.setManualControl(lastManualControl);
    }
}
