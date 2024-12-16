package org.firstinspires.ftc.teamcode.vision;

import android.util.Pair;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class VisionPipeline extends OpenCvPipeline {
    private final VisionConstants.GamePieceColor color;

    public VisionPipeline(VisionConstants.GamePieceColor color) {
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat frame) {
        Mat mask = this.maskGamePiece(frame);
        Pair<Mat, List<RectLocation>> gamePiece = detectGamePiece(mask, VisionConstants.MIN_AREA);
        Mat filledMask = gamePiece.first;
        List<RectLocation> locations = gamePiece.second;

        if(!locations.isEmpty()) {
            RectLocation target = locations.get(0);

            this.displayGamePiece(frame, target.getRect());

            Pair<Double, Direction> centerDistance = this.calculateCenterDistance(frame, target.getCenter().x);
            double yaw = this.getYaw(frame, target.getCenter().x);

            String text = "Distance X: " + centerDistance.first + "px (" + centerDistance.second.name() + ")\nYaw: " + yaw;
            Imgproc.putText(
                    frame,
                    text,
                    new Point(10, 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    this.color.getColor(),
                    2
            );
        }

        return frame;
    }

    private Mat maskGamePiece(Mat frame) {
        Mat blur = new Mat();
        Imgproc.GaussianBlur(frame, blur, new Size(5, 5), 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();

        switch (color) {
            case RED:
                Mat mask1 = new Mat();
                Core.inRange(hsv, VisionConstants.RED_LOWER1, VisionConstants.RED_UPPER1, mask1);

                Mat mask2 = new Mat();
                Core.inRange(hsv, VisionConstants.RED_LOWER2, VisionConstants.RED_UPPER2, mask2);
                Core.bitwise_or(mask1, mask2, mask);
                break;

            case BLUE:
                Core.inRange(hsv, VisionConstants.BLUE_LOWER, VisionConstants.BLUE_UPPER, mask);
                break;

            case YELLOW:
                Core.inRange(hsv, VisionConstants.YELLOW_LOWER, VisionConstants.YELLOW_UPPER, mask);
                break;

            default:
                throw new RuntimeException("Color not found");
        }

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        return mask;
    }

    private Pair<Mat, List<RectLocation>> detectGamePiece(Mat mask, double minArea) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<RectLocation> locations = new ArrayList<>();
        Mat filledMask = new Mat();

        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                Imgproc.drawContours(filledMask, List.of(contour), -1, this.color.getColor());

                Rect rect = Imgproc.boundingRect(contour);

                locations.add(new RectLocation(
                        new Point(
                                rect.x + (rect.width / 2),
                                rect.y + (rect.height / 2)
                        ),
                        rect,
                        area
                ));
            }
        }
        locations.sort(Comparator.comparingDouble(location -> location.area));
        return Pair.create(filledMask, locations);
    }

    private double getYaw(Mat frame, double targetCenterX) {
        int frameWidth = frame.width();
        int frameCenterX = frameWidth / 2;

        double offsetX = targetCenterX - frameCenterX;
        return (offsetX / (frameWidth / 2f)) * (VisionConstants.FOV_HORIZONTAL / 2f);
    }

    private Pair<Double, Direction> calculateCenterDistance(Mat frame, double targetCenterX) {
        int frameWidth = frame.width();

        int frameCenterX = frameWidth / 2;

        double distanceX = targetCenterX - frameCenterX;

        return Pair.create(distanceX, distanceX > 0 ? Direction.RIGHT : Direction.LEFT);
    }

    private void displayGamePiece(Mat frame, Rect rect) {
        Imgproc.rectangle(frame, rect, this.color.getColor(), 4);
    }

    private final class RectLocation {
        private final Point center;
        private final Rect rect;
        private final double area;

        public RectLocation(Point center, Rect rect, double area) {
            this.center = center;
            this.rect = rect;
            this.area = area;
        }

        public Rect getRect() {
            return rect;
        }

        public double getArea() {
            return area;
        }

        public Point getCenter() {
            return center;
        }
    }

    public enum Direction {
        LEFT,
        RIGHT
    }
}
