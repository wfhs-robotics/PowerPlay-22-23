package org.firstinspires.ftc.teamcode.Others;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class SamplePipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    Mat output = new Mat();
    int avg;
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    Mat leftCrop;
    Mat rightCrop;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);

    }

    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToY(input);
        System.out.println("processing requested");
        avg = (int) Core.mean(Y).val[0];
        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!

        Rect leftRect = new Rect(1, 1, 319, 359); //1280x720 total resolution
        Rect rightRect = new Rect(320, 1, 319, 359);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCrCb.submat(leftRect);
        rightCrop = YCrCb.submat(rightRect);

        return input;

    }

    public int getAnalysis() {
        return avg;
    }
}