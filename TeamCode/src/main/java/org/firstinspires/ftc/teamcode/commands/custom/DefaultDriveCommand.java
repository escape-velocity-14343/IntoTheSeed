package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PtoSubsystem;

public class DefaultDriveCommand extends CommandBase {
    MecanumDriveSubsystem drive;
    PtoSubsystem pto;
    DoubleSupplier x, y, rx, heading;

    public DefaultDriveCommand(
            MecanumDriveSubsystem driveSubsystem,
            PtoSubsystem pto,
            DoubleSupplier inputX,
            DoubleSupplier inputY,
            DoubleSupplier inputRx,
            DoubleSupplier robotHeading) {
        this.drive = driveSubsystem;
        this.pto = pto;
        this.x = inputX;
        this.y = inputY;
        this.rx = inputRx;
        this.heading = robotHeading;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (pto != null && pto.isEngaged()) {
            return;
        }
        //Log.i("default drive command", "it is happening");
        drive.driveFieldCentric(
                -x.getAsDouble() + getXModPower(),
                y.getAsDouble() + getYModPower(),
                rx.getAsDouble() + getRModPower(),
                heading.getAsDouble());
    }

    public double getXModPower() {
        return 0.0;
    }

    public double getYModPower() {
        return 0.0;
    }

    public double getRModPower() {
        return 0.0;
    }
}
