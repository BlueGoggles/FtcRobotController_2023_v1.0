package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipeline extends OpenCvPipeline {

    private double cX = 0;
    private double cY = 0;
    private double width = 0;
    private double objectWidthInRealWorldUnits = 0.0;
    private Utility.Color color = null;

    public ColorDetectionPipeline(double objectWidthInRealWorldUnits, Utility.Color color) {

        this.objectWidthInRealWorldUnits = objectWidthInRealWorldUnits;
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Preprocess the frame to detect color regions
        Mat colorMask = preprocessFrame(input);

        // Find contours of the detected color regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest color contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        }

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerColor = null;
        Scalar upperColor = null;

        if (this.color == Utility.Color.BLUE) {
            lowerColor = new Scalar(0, 125,125);
            upperColor = new Scalar(60, 255, 255);
        } else if (this.color == Utility.Color.RED) {
            lowerColor = new Scalar(100, 50, 50);
            upperColor = new Scalar(180, 255, 255);
        } else {
            return null;
        }

        Mat colorMask = new Mat();
        Core.inRange(hsvFrame, lowerColor, upperColor, colorMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

        return colorMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public double getDistance(double width){
        return (this.objectWidthInRealWorldUnits * Constants.CAMERA_FOCAL_LENGTH) / width;
    }

    public double getcX() {
        return cX;
    }

    public double getcY() {
        return cY;
    }

    public double getWidth() {
        return width;
    }
}