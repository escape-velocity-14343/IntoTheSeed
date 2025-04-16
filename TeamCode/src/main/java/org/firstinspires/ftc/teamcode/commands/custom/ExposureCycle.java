package org.firstinspires.ftc.teamcode.commands.custom;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class ExposureCycle extends CommandBase {
    VisionSubsystem cam;
    int exposureMillis = 10;
    boolean lastExposureSuccess = false;
    boolean cameraPooer = false;
    public ExposureCycle(VisionSubsystem camera, boolean whichCamera) {
        cam = camera;
        cameraPooer = whichCamera;
        addRequirements(cam);
    }
    @Override
    public void initialize() {
        cam.setCam(cameraPooer);
        cam.setExposure(exposureMillis);
    }
    @Override
    public void execute() {
        if (lastExposureSuccess) {
            cam.saveFrame("Exposure: " + exposureMillis + "ms, Switch to Chassis: " + cameraPooer);
            exposureMillis+=100;
        }
        lastExposureSuccess = cam.setExposure(exposureMillis);
    }
    @Override
    public void end(boolean wasInterrupted) {
        //reset to original exposure (ftcdash)
        cam.setExposure();
    }
    @Override
    public boolean isFinished() {
        return exposureMillis >= 1000;
    }
}