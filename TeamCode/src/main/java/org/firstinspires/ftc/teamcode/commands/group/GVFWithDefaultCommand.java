package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.lib.path.spline.Spline;

import java.util.Objects;
import java.util.function.Supplier;

public class GVFWithDefaultCommand extends CommandBase {
    private Supplier<Spline[]> targetSupplier;
    private boolean useTargetSupplier = false;
    private DefaultGVFCommand gtpc;
    private Spline[] splines;
    private Supplier<Spline[]> splineSupplier;

    public GVFWithDefaultCommand(DefaultGVFCommand gtpc, Spline... splines) {
        this.splines = splines;
        this.gtpc = gtpc;
    }

    public GVFWithDefaultCommand(DefaultGVFCommand gtpc, Supplier<Spline[]> splineSupplier) {
        this.splineSupplier = splineSupplier;
        this.gtpc = gtpc;
    }

    public GVFWithDefaultCommand(DefaultGVFCommand gtpc, double tol, double hTol, Supplier<Spline[]> splineSupplier) {
        this.splineSupplier = splineSupplier;
        this.gtpc = gtpc;
        gtpc.setTolerances(tol, hTol);
    }

    public GVFWithDefaultCommand(DefaultGVFCommand gtpc, double tol, double hTol, Spline... splines) {
        this.splines = splines;
        this.gtpc = gtpc;
        gtpc.setTolerances(tol, hTol);
    }

    public void initialize() {
        if (Objects.nonNull(splineSupplier)) {
            splines = splineSupplier.get();
        }
        gtpc.setSplines(splines);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean wasInterrupted) {
        Log.i(
                "%1",
                "gtp finished "
                        + gtpc.target.getX()
                        + " "
                        + gtpc.target.getY()
                        + " "
                        + gtpc.target.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        //gtpc.setTolerances(3, 4);
        return gtpc.isDone();
    }
}
