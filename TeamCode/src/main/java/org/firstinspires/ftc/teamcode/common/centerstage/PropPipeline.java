package org.firstinspires.ftc.teamcode.common.centerstage;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;


import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class PropPipeline implements VisionProcessor {
    private Mat testMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat finalMat = new Mat();
    // TODO: Tune these threshold values
    double redThreshold = 0.5;
    double blueThreshold = 0.5;
    double threshold = 0;
    private Side location = Side.LEFT;

    private final Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
    private final Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

    private final Scalar highHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
    private final Scalar highHSVRedUpper = new Scalar(180, 255, 255);

    private final Scalar lowHSVBlueLower = new Scalar(110, 50, 50);
    private final Scalar highHSVBlueUpper = new Scalar(130, 255, 255);



    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(20, 20),
            new Point(50, 50)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(100, 100),
            new Point(100, 100)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        threshold = (Globals.COLOR == Side.RED) ? redThreshold : blueThreshold;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if (Globals.COLOR == Side.RED) {
            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);
        }
        else {
            Core.inRange(testMat, lowHSVBlueLower, lowHSVBlueLower, lowMat);
            Core.inRange(testMat, highHSVBlueUpper, highHSVBlueUpper, highMat);
        }

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]


        if (averagedLeftBox > threshold) {        //Must Tune Threshold
            location = Side.LEFT;
        } else if (averagedRightBox > threshold) {
            location = Side.CENTER;
        } else {
            location = Side.RIGHT;
        }

        finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return null;            //You do not return the original mat anymore, instead return null

    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(LEFT_RECTANGLE, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(RIGHT_RECTANGLE, scaleBmpPxToCanvasPx), rectPaint);
    }


    public Side getPropPosition() {
        return this.location;
    }
}