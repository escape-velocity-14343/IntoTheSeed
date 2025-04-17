package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.SlideKinematics;
import org.firstinspires.ftc.teamcode.vision.ColorBlobLocatorProcessorMulti;
import org.firstinspires.ftc.teamcode.vision.ColorRange;
import org.firstinspires.ftc.teamcode.vision.GlowUpPipeline;
import org.firstinspires.ftc.teamcode.vision.ImageRegion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Config
public class VisionSubsystem extends SubsystemBase {
    public static Scalar minimumRed1 = new Scalar(0, 70, 50);
    public static Scalar maximumRed1 = new Scalar(12, 255, 255);

    public static Scalar minimumRed2 = new Scalar(168, 70, 50);
    public static Scalar maximumRed2 = new Scalar(180, 255, 255);

    public static Scalar minimumBlue = new Scalar(100, 150, 100);
    public static Scalar maximumBlue = new Scalar(140, 255, 255);

    public static Scalar minimumYellow = new Scalar(13, 40, 80);
    public static Scalar maximumYellow = new Scalar(50, 255, 255);

    public boolean useGlowUp = false;

    public static int exposureMillis = 800;
    public static int minContourArea = 200;
    public static int maxContourArea = 13000;
    public static double alpha = 1; //gain scalar
    public static double beta = 0; //brightness offset
    DoubleSupplier extensionSupplier = () -> Double.MAX_VALUE;
    DoubleSupplier pivotSupplier;
    Supplier<Pose2d> pos;


    ColorBlobLocatorProcessorMulti colorLocator, closeLocator;
    GlowUpPipeline glowUp;
    private double pixelPos = 0;

    private Vector2d samplePos = new Vector2d();
    private ArrayList<Vector2d> samplePoses = new ArrayList<>();

    Telemetry telemetry;
    VisionPortal visionPortal;
    private final WebcamName chassisCam;
    private final WebcamName slideCam;
    //useChassisCamera is only intended to be used for init logging don't base anything off of it
    private boolean useChassisCamera = true;
    public static boolean defaultSetCamValue = true;
    private double angle = 0;

    private boolean possibleSample = false;
    private boolean confirmedSample = false;
    public static int erodeKernel = 15;
    Optional<ColorBlobLocatorProcessor.Blob> largestBlob = Optional.empty();


    public VisionSubsystem(HardwareMap hMap, Telemetry telemetry) {
        glowUp = new GlowUpPipeline();
        colorLocator = new ColorBlobLocatorProcessorMulti(
                new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, new Scalar(13, 60, 60), new Scalar(50, 255, 255)),
                ImageRegion.asImageCoordinates(0, 0, VisionConstants.width, VisionConstants.height),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                5,
                2,
                false,
                -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252),
                new Point[]{
                        new Point(0, VisionConstants.minHeight),
                        new Point(VisionConstants.width, VisionConstants.minHeight),
                        new Point(VisionConstants.width, VisionConstants.height),
                        new Point(0, VisionConstants.height)
                }
        );
        closeLocator = new ColorBlobLocatorProcessorMulti(
                new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, new Scalar(13, 60, 60), new Scalar(50, 255, 255)),
                ImageRegion.asImageCoordinates(0, 0, VisionConstants.width, VisionConstants.height),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                erodeKernel,
                2,
                false,
                -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252),
                new Point[]{
                        new Point(0, 0),
                        new Point(VisionConstants.width, 0),
                        new Point(VisionConstants.width, VisionConstants.height),
                        new Point(0, VisionConstants.height)
                }
        );


        // add yellow colors (same for all alliances)
        colorLocator.addColors(new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, minimumYellow, maximumYellow));
        closeLocator.addColors(new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, minimumYellow, maximumYellow));

        switch (AutoConstants.alliance) {
            case RED:
                colorLocator.addColors(new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, minimumRed1, maximumRed1));
                colorLocator.addColors(new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, minimumRed2, maximumRed2));
                closeLocator.addColors(new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, minimumRed1, maximumRed1));
                closeLocator.addColors(new org.firstinspires.ftc.teamcode.vision.ColorRange(ColorSpace.HSV, minimumRed2, maximumRed2));
                break;
            case BLUE:
                colorLocator.addColors(new ColorRange(ColorSpace.HSV, minimumBlue, maximumBlue));
                closeLocator.addColors(new ColorRange(ColorSpace.HSV, minimumBlue, maximumBlue));
                break;
        }

        chassisCam = hMap.get(WebcamName.class, VisionConstants.chassisCameraName);
        slideCam = hMap.get(WebcamName.class, VisionConstants.slideCameraName);
        CameraName doubleCam = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(chassisCam, slideCam);
        /*int viewportid = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)[0];
        if (name.equals(VisionConstants.slideCameraName)) {
            Log.i("Viewport", "is this working");
            viewportid = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)[1];
        }*/
        if (useGlowUp) {
            visionPortal = new VisionPortal.Builder()
                    .addProcessors(glowUp, colorLocator, closeLocator)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(doubleCam)
                    .enableLiveView(true)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .addProcessors(colorLocator, closeLocator)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(doubleCam)
                    .enableLiveView(true)
                    .build();
        }

        setCam(defaultSetCamValue);

        setEnabled(true);
//        waitForSetExposure(1000, 1000);

        this.telemetry = telemetry;
    }
    public void setSuppliers(DoubleSupplier extensionSupplier, DoubleSupplier pivotSupplier, Supplier<Pose2d> pos) {
        this.extensionSupplier = extensionSupplier;
        this.pivotSupplier = pivotSupplier;
        this.pos = pos;
    }

    public void setExtensionSupplier(DoubleSupplier extensionSupplier) {
        this.extensionSupplier = extensionSupplier;
    }

    public void setCamWithTimeout(long timeoutMs, int maxAttempts, boolean switchToChassis) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("Camera Stream", String.format("Waiting for camera stream, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setCam(switchToChassis)){
                Log.i("Camera Stream", "Camera stream was opened successfully!");
                return;
            }
        }
        Log.e("Camera Stream", "While waiting for camera stream, timeout or max attempts exited. This probably means that exposure is gonna have a hard time");
    }

    public boolean setCam(boolean switchToChassis) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(switchToChassis ? this.chassisCam : this.slideCam);
            visionPortal.setProcessorEnabled(colorLocator, switchToChassis);
            visionPortal.setProcessorEnabled(closeLocator, !switchToChassis);
            if (!switchToChassis) {
                samplePos = null;
            }
            return true;
        } else {
            return false;
        }
        /*if (streaming) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                visionPortal.resumeStreaming();
            }
        } else if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
        }*/
    }

    @Override
    public void periodic() {
        //telemetry.addData("Is color process", visionPortal.getProcessorEnabled(colorLocator));
        //telemetry.addData("Is close process", visionPortal.getProcessorEnabled(closeLocator));

        pixelPos = 0;

        if (visionPortal.getProcessorEnabled(colorLocator)) {
            possibleSample = false;
            confirmedSample = false;

            samplePoses = new ArrayList<>();

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(minContourArea, maxContourArea, blobs);
            int dist = 10000;
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);


            if (!blobs.isEmpty()) {
                for (int i = 0; i < Math.min(blobs.size(), 3); i++) {
                    samplePoses.add(new Vector2d(blobs.get(i).getBoxFit().center.x, blobs.get(i).getBoxFit().center.y));
                }
                RotatedRect blob = blobs.get(0).getBoxFit();
                pixelPos = (int) (160 - blob.center.x);
                samplePos = new Vector2d(blob.center.x, blob.center.y);
                angle = blob.angle;
                if (blob.size.width < blob.size.height) {
                    angle -= 90;
                }
            }
        } else if (visionPortal.getProcessorEnabled(closeLocator)) {
            List<ColorBlobLocatorProcessor.Blob> ogBlobs = closeLocator.getBlobs();

            if (ogBlobs.isEmpty()){
                return;
            }

            List<ColorBlobLocatorProcessor.Blob> blobs = new LinkedList<>();
            //Pose2d cameraPos = SlideKinematics.getRCCameraPos(Rotation2d.fromDegrees(pivotSupplier.getAsDouble()), extensionSupplier.getAsDouble());
            for (ColorBlobLocatorProcessor.Blob blob : ogBlobs){
                try {

                    blobs.add(blob);
                } catch (NullPointerException e) {
                    Log.i("Null pointer exception", "Blob.size is null");
                }
            }

            double dist = 10000;
            double centerDist = 10000;

            largestBlob = Optional.empty();

            if (!blobs.isEmpty()) {

                ColorBlobLocatorProcessor.Util.filterByArea(minContourArea, maxContourArea, blobs);
                ColorBlobLocatorProcessor.Util.filterByAspectRatio(1.75, 5, blobs);

                // TODO: implement density filtering
                //ColorBlobLocatorProcessor.Util.filterByDensity();

                if (blobs.isEmpty()) {
                    samplePos = null;
                    return;
                }

                ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);


                largestBlob = Optional.of(blobs.get(0));

                double[] weights = new double[blobs.size()];

                // weight by size (10%)
                for (int i = 0; i < blobs.size(); i++) {
                    weights[i] = 0.1 * (double) (blobs.size() - i) / blobs.size();
                }

                if (Objects.nonNull(samplePos)) {

                    // make dist our min dist
                    for (int i = 0; i < blobs.size(); i++) {
                        Point center = blobs.get(i).getBoxFit().center;
                        double newDist = Math.hypot(samplePos.getX() - center.x, samplePos.getY() - center.y);
                        if (newDist < dist) {
                            dist = newDist;
                        }
                    }

                    // weight by distance (75%)
                    for (int i = 0; i < blobs.size(); i++) {
                        Point center = blobs.get(i).getBoxFit().center;
                        double newDist = Math.hypot(samplePos.getX() - center.x, samplePos.getY() - center.y);
                        weights[i] += dist * 0.75 / newDist;
                    }
                }

                // weight by distance from center (15%)

                for (int i = 0; i < blobs.size(); i++) {
                    Point center = blobs.get(i).getBoxFit().center;
                    double newDist = Math.hypot(VisionConstants.xOffset - center.x, VisionConstants.yOffset - center.y);
                    if (newDist < centerDist) {
                        centerDist = newDist;
                    }
                }

                for (int i = 0; i < blobs.size(); i++) {
                    Point center = blobs.get(i).getBoxFit().center;
                    double newDist = Math.hypot(VisionConstants.xOffset - center.x, VisionConstants.yOffset - center.y);
                    weights[i] += centerDist * 0.15 / newDist;
                }


                // push away from edge
                if (extensionSupplier.getAsDouble() < VisionConstants.submersibleIntakeWeightThresholdInches) {
                    for (int i = 0; i < blobs.size(); i++) {
                        Point center = blobs.get(i).getBoxFit().center;
                        weights[i] += center.x / VisionConstants.width * 1.3;
                    }
                }


                double maxWeight = -1;
                int index = 0;
                for (int i = 0; i < weights.length; i++) {
                    if (weights[i] > maxWeight) {
                        maxWeight = weights[i];
                        index = i;
                    }
                }

                pixelPos = dist;
                RotatedRect blob = blobs.get(index).getBoxFit();
                Log.i("vision pooopy", "blob size puyallup: " + blob.size.area());
                possibleSample = blob.size.area()>VisionConstants.minSampleArea;
                pixelPos = (int) (160 - blob.center.x);
                samplePos = new Vector2d(blob.center.x, blob.center.y);
                angle = blob.angle;
                if (blob.size.width < blob.size.height) {
                    angle -= 90;
                }
                confirmedSample = Math.hypot(VisionConstants.xOffset - blob.center.x, VisionConstants.yOffset - blob.center.y)<VisionConstants.visionEndThreshold;
                //angle += 90;
                //angle = AngleUnit.normalizeDegrees(angle);
            } else {
                samplePos = null;
            }
        }
    }

    public Optional<ColorBlobLocatorProcessor.Blob> getLargestBlob(){
        return largestBlob;
    }
    public boolean getPossibilityForSampleExistingInThisGivenMomentOfTimeAndSpace() {
        return possibleSample;
    }
    public boolean isConfirmedSample() {
        return confirmedSample;
    }

    public Vector2d getSamplePos() {
        return samplePos;
    }
    public Pose2d getSamplePose() {
        if (Objects.isNull(samplePos)) {
            return null;
        }
        return new Pose2d(samplePos.getX(), samplePos.getY(), Rotation2d.fromDegrees(angle));
    }

    public ArrayList<Vector2d> getSamplePoses() {
        return samplePoses;
    }


    public void setEnabled(boolean enable) {
        visionPortal.setProcessorEnabled(colorLocator, enable);
    }

    public double getPixelPos() {
        return pixelPos;
    }

    /**
     * @return whether the set was successful or not
     */
    public boolean setExposure(int exposure) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl control = visionPortal.getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("camera", "exposure: " + control.getExposure(TimeUnit.MILLISECONDS));
        return control.setExposure(exposure, TimeUnit.MILLISECONDS);
    }

    public boolean setExposure() {
        return setExposure(exposureMillis);
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts) {
        return waitForSetExposure(timeoutMs, maxAttempts, exposureMillis);
    }

    public void setOnlyYellow(boolean onlyYellow) {
        colorLocator.onlyFirstColor = onlyYellow;
    }

    private boolean waitForSetExposureInternal(long timeoutMs, int maxAttempts, int exposure, String cameraNameLog) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i(String.format("Camera %s", cameraNameLog), String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure)) {
                Log.i("Set Exposure Succeeded: ", String.format("For Camera %s", cameraNameLog));
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed");
        return false;
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure) {
        setCam(true);
        boolean a = waitForSetExposureInternal(timeoutMs, maxAttempts, exposure, VisionConstants.chassisCameraName);
        setCam(false);
        boolean b = waitForSetExposureInternal(timeoutMs, maxAttempts, exposure, VisionConstants.slideCameraName);
        return a && b;
    }

    public void saveFrame(String name) {
        visionPortal.saveNextFrameRaw(name);
    }

    public void setUseGlowUp(boolean state){
        useGlowUp = state;
    }

    public Optional<sample> getClosestChassis() {
        if (visionPortal.getProcessorEnabled(colorLocator)) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(minContourArea, 20000, blobs);

            if (!blobs.isEmpty()) {
                Optional<sample> selected = Optional.empty();
                Double closestDistance = Double.MAX_VALUE;
                for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                    Point center = blob.getBoxFit().center;
                    Vector2d target = VisionConstants.target;
                    Double norm = Math.sqrt(Math.pow(center.x - target.getX(), 2) + Math.pow(center.y - target.getY(), 2));

                    if (closestDistance > norm){
                        closestDistance = norm;
                        if (blob.getBoxFit().size.width < blob.getBoxFit().size.height) {
                            selected = Optional.of(new sample(blob.getBoxFit().angle-90, center.x, center.y));
                        }
                        else{
                            selected = Optional.of(new sample(blob.getBoxFit().angle, center.x, center.y));
                        }
                    }
                }
                return selected;
            }
        }
        return Optional.empty();
    }

    public static class sample{
        public double angle = 0;
        public double x = 0;
        public double y = 0;

        public sample(double angle, double x, double y){
            this.angle = angle;
            this.x = x;
            this.y = y;
        }
    }
}


