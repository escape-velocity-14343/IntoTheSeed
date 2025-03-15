package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

@Config
public class VisionSubsystem extends SubsystemBase {
    public BooleanSupplier isRed = () -> false;

    public static int lowAreaFilter = 600;
    public static int AreaFilterYellow = 1200;
    public static int highAreaFilter = 2500;
    public static double lowRatioFilter = 1.3;
    public static double RatioFilterYellow = 1.5;
    public static double highRatioFilter = 2.8;

    public static int exposureMillis = 50;
    ColorRange blue = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(0, 0, 0),
            new Scalar(0, 0, 0)
    );

//    ColorRange blue = new ColorRange(
//            ColorSpace.HSV,
//            new Scalar(0, 0, 0),
//            new Scalar(0, 0, 0)
//    );

    ColorRange red = new ColorRange(
            ColorSpace.HSV,
            new Scalar(0, 0, 0),
            new Scalar(0, 0, 0)
    );

    ColorRange yellow = new ColorRange(
            ColorSpace.HSV,
            new Scalar(0, 0, 0),
            new Scalar(0, 0, 0)
    );

    ColorBlobLocatorProcessor.Builder allianceLocatorProcessBuilder = new ColorBlobLocatorProcessor.Builder()
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.entireFrame())
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
            .setBlurSize(1)
            .setErodeSize(4);

    ColorBlobLocatorProcessor allianceLocatorProcessor;
    ColorBlobLocatorProcessor yellowLocatorProcessor = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(yellow)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, 0.8, 0.8, -0.8))
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
            .setBlurSize(1)
            .setErodeSize(6)
            .build();

    Telemetry telemetry;
    VisionPortal visionPortal;
    ExposureControl control;


    public VisionSubsystem(HardwareMap hMap, BooleanSupplier isRed, Telemetry telemetry){
        CameraName slideCamera = hMap.get(WebcamName.class, VisionConstants.slideCameraName);

        this.isRed = isRed;
        this.telemetry = telemetry;

        if (isRed.getAsBoolean()){
            allianceLocatorProcessBuilder.setTargetColorRange(red);
        }
        else{
            allianceLocatorProcessBuilder.setTargetColorRange(blue);
        }

        ColorBlobLocatorProcessor.BlobFilter areaFilter =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, lowAreaFilter, highAreaFilter);
        ColorBlobLocatorProcessor.BlobFilter ratioFilter =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, lowRatioFilter, highRatioFilter);
        ColorBlobLocatorProcessor.BlobSort largestSort =
                new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING);

        allianceLocatorProcessor.addFilter(areaFilter);
        allianceLocatorProcessor.addFilter(ratioFilter);
        allianceLocatorProcessor.setSort(largestSort);

        allianceLocatorProcessor = allianceLocatorProcessBuilder.build();

        ColorBlobLocatorProcessor.BlobFilter areaFilterYellow =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, AreaFilterYellow, highAreaFilter);
        ColorBlobLocatorProcessor.BlobFilter ratioFilterYellow =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, RatioFilterYellow, highRatioFilter);

        yellowLocatorProcessor.addFilter(areaFilterYellow);
        yellowLocatorProcessor.addFilter(ratioFilterYellow);
        yellowLocatorProcessor.setSort(largestSort);

        visionPortal = new VisionPortal.Builder()
                .setCamera(slideCamera)
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(yellowLocatorProcessor)
                .addProcessor(allianceLocatorProcessor)
                .build();

        control = visionPortal.getCameraControl(ExposureControl.class);
    }

    /**
     * intended entry point for teleop claw angle
     * @return
     */
    public Optional<ColorBlobLocatorProcessor.Blob> getClosestBlobCenter(){
        List<ColorBlobLocatorProcessor.Blob> yellowDetected = yellowLocatorProcessor.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> allianceDetected = allianceLocatorProcessor.getBlobs();

        List<ColorBlobLocatorProcessor.Blob> merged = new LinkedList<>();
        merged.addAll(yellowDetected);
        merged.addAll(allianceDetected);

        if (merged.isEmpty()){
            return Optional.empty();
        }

        return getClosestBlobCenterXY(merged);
    }

    /**
     * Purely on the x axis, cartesian
     * @param blobs
     * @return
     */
    public Optional<ColorBlobLocatorProcessor.Blob> getClosestBlobCenterX(List<ColorBlobLocatorProcessor.Blob> blobs){
        if (blobs.isEmpty()){
            return Optional.empty();
        }
        Optional<ColorBlobLocatorProcessor.Blob> closestBlob = Optional.empty();
        for (ColorBlobLocatorProcessor.Blob blob : blobs){
            if (!closestBlob.isPresent()){
                closestBlob = Optional.of(blob);
            }
            else if(distanceFromCenterPixelX(blob) < distanceFromCenterPixelX(closestBlob.get())){
                closestBlob = Optional.of(blob);
            }
        }

        return closestBlob;
    }

    /**
     * Limited to a 25% x 25% region in the center of the frame
     * @param blobs
     * @return
     */
    public Optional<ColorBlobLocatorProcessor.Blob> getClosestBlobCenterXY(List<ColorBlobLocatorProcessor.Blob> blobs){
        if (blobs.isEmpty()){
            return Optional.empty();
        }
        Optional<ColorBlobLocatorProcessor.Blob> closestBlob = Optional.empty();
        for (ColorBlobLocatorProcessor.Blob blob : blobs){
            if (!closestBlob.isPresent()){
                closestBlob = Optional.of(blob);
            }
//            else if(distanceFromCenterPixelX(blob) > VisionConstants.width/8 || distanceFromCenterPixelY(blob) > VisionConstants.height/8){
//                continue;
//            }
            else if (!withinPixelRegion(blob, 500, 900, 0, 400)){
                continue;
            }
            else if(distanceFromCenterPixelX(blob) < distanceFromCenterPixelX(closestBlob.get())){
                closestBlob = Optional.of(blob);
            }
        }

        return closestBlob;
    }

    /**
     * x1 < x2, y1 < y2
     * @param blob
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    public boolean withinPixelRegion(ColorBlobLocatorProcessor.Blob blob, int x1, int y1, int x2, int y2){
        Point fit = blob.getBoxFit().center;
        return ((fit.x < x1 || fit.x > x2) || (fit.y < y1 || fit.y > y2));
    }

    public Double distanceFromCenterPixelX(ColorBlobLocatorProcessor.Blob blob){
        return Math.signum(VisionConstants.width-blob.getBoxFit().center.x);
    }

    public Double distanceFromCenterPixelY(ColorBlobLocatorProcessor.Blob blob){
        return Math.signum(VisionConstants.height-blob.getBoxFit().center.y);
    }

    public void stopStream(){
        visionPortal.stopStreaming();
    }

    public void restartStream(){
        visionPortal.resumeStreaming();
    }

    public Double getSampleAngle(ColorBlobLocatorProcessor.Blob blob){
        final RotatedRect boxFit = blob.getBoxFit();

        return boxFit.angle;
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts) {
        return waitForSetExposure(timeoutMs, maxAttempts, exposureMillis);
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("camera", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure)) {
                Log.i("camera", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed");
        return false;
    }

    public boolean waitForSetGain(long timeoutMs, int maxAttempts, int gain) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("camera", String.format("Attempting to set camera gain, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setGain(gain)) {
                Log.i("camera", "Set gain succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set gain failed");
        return false;
    }

    /**
     * @return whether the set was successful or not
     */
    public boolean setExposure(int exposure) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        control.setMode(ExposureControl.Mode.Manual);
        Log.i("camera", "exposure: " + control.getExposure(TimeUnit.MILLISECONDS));
        return control.setExposure(exposure, TimeUnit.MILLISECONDS);
    }

    /**
     * @return whether the set was successful or not
     * @param gain
     * @return
     */
    public boolean setGain(int gain){
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        Log.i("camera", "exposure: " + control.getExposure(TimeUnit.MILLISECONDS));
        return gainControl.setGain(gain);
    }

    public void saveFrame(String name) {
        visionPortal.saveNextFrameRaw(name);
    }
}
