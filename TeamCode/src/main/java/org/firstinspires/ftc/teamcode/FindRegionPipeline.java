package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FindRegionPipeline extends OpenCvPipeline {

    private Mat YCrCb = new Mat();
    private Mat leftCrop;
    private Mat rightCrop;
    private double leftAvgFinal;
    private double rightAvgFinal;
    private Mat output = new Mat();
    private Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    private Utility.Color color = null;

    public FindRegionPipeline(Utility.Color color) {
        this.color = color;
    }

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(165, 160, 105, 140);
        Rect rightRect = new Rect(370, 160, 105, 95);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCrCb.submat(leftRect);
        rightCrop = YCrCb.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, this.color.getCode());
        Core.extractChannel(rightCrop, rightCrop, this.color.getCode());

        Scalar leftAvg = Core.mean(leftCrop);
        Scalar rightAvg = Core.mean(rightCrop);

        leftAvgFinal = leftAvg.val[0];
        rightAvgFinal = rightAvg.val[0];

        return output;
    }

    public double getLeftAvgFinal() {
        return this.leftAvgFinal;
    }

    public double getRightAvgFinal() {
        return this.rightAvgFinal;
    }
}