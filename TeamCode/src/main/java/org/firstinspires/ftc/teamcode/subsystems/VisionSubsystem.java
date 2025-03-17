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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
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

    public static int lowAreaFilter = 100; //try like 400-500 later
    public static int AreaFilterYellow = 100;
    public static int highAreaFilter = 50000;
    public static double lowRatioFilter = 1;
    public static double RatioFilterYellow = 3;
    public static double highRatioFilter = 3;

    public static int exposureMillis = 50;
    //H: 0-179, S: 0-255, V: 0-255

    // YCrCb (24, 154, 122)
    ColorRange blue = new ColorRange(
            ColorSpace.HSV,
            new Scalar(100, 50, 5),
            new Scalar(140, 255, 255)
    );

//    ColorRange blue = new ColorRange(
//            ColorSpace.HSV,
//            new Scalar(0, 0, 0),
//            new Scalar(0, 0, 0)
//    );

    //348°/2, 36%, 74%
    // ->

//    public static final ColorRange RED = new ColorRange(
//            ColorSpace.YCrCb,
//            new Scalar( 32, 176,  0),
//            new Scalar(255, 255, 132)
//    );
//    ColorRange red = new ColorRange(
//            ColorSpace.RGB,
//            new Scalar(50, 50, 50),
//            new Scalar(255, 200, 200)
//    );
//    ColorRange red = new ColorRange(
//            ColorSpace.YCrCb,
//            new Scalar(32, 190, 30),
//            new Scalar(255, 255, 132));

    //53°/2, 13%, 100%
    ColorRange yellow = new ColorRange(
            ColorSpace.HSV,
            new Scalar(10, 120, 120),
            new Scalar(50, 255, 255)
    );

    ColorBlobLocatorProcessor.Builder allianceLocatorProcessBuilder = new ColorBlobLocatorProcessor.Builder()
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.entireFrame())
            .setRoi(ImageRegion.asImageCoordinates(0, 0, VisionConstants.width, VisionConstants.height))
            .setRoiColor(0);
//            .setBlurSize(1)
//            .setErodeSize(4);

    ColorBlobLocatorProcessor allianceLocatorProcessor;
    ColorBlobLocatorProcessor yellowLocatorProcessor = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(yellow)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asImageCoordinates(0, 0, VisionConstants.width, VisionConstants.height))
            .setRoiColor(3)
//            .setBlurSize(1)
//            .setErodeSize(6)
            .build();

    Telemetry telemetry;
    VisionPortal visionPortal;
    CameraName cameraName;


    public VisionSubsystem(HardwareMap hMap, String name, BooleanSupplier isRed, Telemetry telemetry){
        cameraName = hMap.get(WebcamName.class, name);

        this.isRed = isRed;
        this.telemetry = telemetry;

        if (isRed.getAsBoolean()){
            allianceLocatorProcessBuilder.setTargetColorRange(ColorRange.RED);
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

        allianceLocatorProcessor = allianceLocatorProcessBuilder.build();

//        allianceLocatorProcessor.addFilter(areaFilter);
//        allianceLocatorProcessor.addFilter(ratioFilter);
        allianceLocatorProcessor.setSort(largestSort);

        ColorBlobLocatorProcessor.BlobFilter areaFilterYellow =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, AreaFilterYellow, highAreaFilter);
        ColorBlobLocatorProcessor.BlobFilter ratioFilterYellow =
                new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, RatioFilterYellow, highRatioFilter);

        yellowLocatorProcessor.addFilter(areaFilterYellow);
//        yellowLocatorProcessor.addFilter(ratioFilterYellow);
        yellowLocatorProcessor.setSort(largestSort);

        visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(new Size(VisionConstants.width, VisionConstants.height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(yellowLocatorProcessor)
                .addProcessor(allianceLocatorProcessor)
                .build();

//        waitForSetEverything(3000, 10000, 50, 1000, 1000);
        waitForSetExposure(2000, 10000, 30);
        waitForSetWhiteBalance(2000, 10000, 3000);
//        waitForSetGain(2000, 10000, 2000);
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

    public void waitForSetEverything(long timeoutMs, int maxAttempts, int exposure, int gain, int whiteBalance){
        waitForSetExposure(timeoutMs, maxAttempts, exposure);
//        waitForSetGain(timeoutMs, maxAttempts, gain);
        waitForSetWhiteBalance(timeoutMs, maxAttempts, whiteBalance);
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts) {
        return waitForSetExposure(timeoutMs, maxAttempts, exposureMillis);
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i(cameraName.toString(), String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure)) {
                Log.i(cameraName.toString(), "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e(cameraName.toString(), "Set exposure failed");
        return false;
    }

    @Deprecated
    public boolean waitForSetGain(long timeoutMs, int maxAttempts, int gain) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i(cameraName.toString(), String.format("Attempting to set camera gain, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setGain(gain)) {
                Log.i(cameraName.toString(), "Set gain succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e(cameraName.toString(), "Set gain failed");
        return false;
    }

    public boolean waitForSetWhiteBalance(long timeoutMs, int maxAttempts, int whiteBalance) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i(cameraName.toString(), String.format("Attempting to set camera white balance, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setWhiteBalance(whiteBalance)) {
                Log.i(cameraName.toString(), "Set white balance succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e(cameraName.toString(), "Set white balance failed");
        return false;
    }

    /**
     * @return whether the set was successful or not
     */
    public boolean setExposure(int exposure) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        Log.i(cameraName.toString(), "exposure: " + exposureControl.getExposure(TimeUnit.MILLISECONDS));
        return exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
    }

    @Deprecated
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

        Log.i(cameraName.toString(), "gain: " + gainControl.getGain());
        return gainControl.setGain(gain);
    }

    public boolean setWhiteBalance(int gain){
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        Log.i(cameraName.toString(), "white balance: " + whiteBalanceControl.getWhiteBalanceTemperature());
        return whiteBalanceControl.setWhiteBalanceTemperature(gain);
    }

    public Optional<Integer> getMaxWhiteBalance(){
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return Optional.empty();
        }

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        return Optional.of(whiteBalanceControl.getMaxWhiteBalanceTemperature());
    }

    public Optional<Integer> getMinWhiteBalance(){
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return Optional.empty();
        }

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        return Optional.of(whiteBalanceControl.getMinWhiteBalanceTemperature());
    }

    public void saveFrame(String name) {
        visionPortal.saveNextFrameRaw(name);
    }


    public void turnOnStreaming(boolean enabled){
        if(enabled){
            visionPortal.resumeStreaming();
        }
        else{
            visionPortal.stopStreaming();
        }
    }
}
