package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class GlowUpPipeline implements VisionProcessor {
    private double alpha = 1; //scalar
    private double beta = 0; //constant gain
    private double contrast = 40; //default is 40

    public GlowUpPipeline(double alpha, double beta, double contrast){
        this.alpha = alpha;
        this.beta = beta;
        this.contrast = contrast;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //Gain and brightness
        frame.convertTo(frame, -1, alpha, beta);

        //Increases contrast
        Mat lab = new Mat();
        Imgproc.cvtColor(frame, lab, Imgproc.COLOR_RGB2Lab);

        List<Mat> labChannels = new ArrayList<>();
        Core.split(lab, labChannels);

        Mat lChannel = labChannels.get(0);
        CLAHE clahe = Imgproc.createCLAHE(contrast);
        clahe.apply(lChannel, lChannel);

        Core.merge(labChannels, lab);
        Imgproc.cvtColor(lab, frame, Imgproc.COLOR_Lab2RGB);

        lab.release();
        for (Mat mat : labChannels){
            mat.release();
        }
        lChannel.release();
        clahe = null; // so that the GC will clean it up

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
