package org.firstinspires.ftc.teamcode.common.centerstage;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private Mat testMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat finalMat = new Mat();
    // TODO: Tune these threshold values
    double redThreshold = 2.5;
    double blueThreshold = 0.2;
    double threshold = 0;
    private Side location = Side.LEFT;

    private final Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
    private final Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

    private final Scalar highHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
    private final Scalar highHSVRedUpper = new Scalar(180, 255, 255);

    private final Scalar lowHSVBlueLower = new Scalar(110, 60, 50);
    private final Scalar highHSVBlueUpper = new Scalar(120, 255, 255);



    static final Rect LEFT_RECTANGLE = new Rect(0, 161, 190, 169);

    static final Rect CENTER_RECTANGLE = new Rect(441, 175, 144, 144);

    Telemetry telemetry;
    int color;
    public PropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (Globals.COLOR == Side.RED) {
            // RED
            color = Imgproc.COLOR_RGB2HSV;
        }
        else {
            // Blue
            color = Imgproc.COLOR_RGB2HSV_FULL;
        }
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, color);

        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / CENTER_RECTANGLE.area() / 255; //Makes value [0,1]

        telemetry.addData("averagedLeftBox: ", averagedLeftBox);
        telemetry.addData("averagedRightBox: ", averagedRightBox);
        telemetry.addData("threshold: ", threshold);
        telemetry.update();
        if (averagedLeftBox > threshold) {        //Must Tune Threshold
            location = Side.LEFT;
            Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(255, 255, 255));
        } else if (averagedRightBox > threshold) {
            location = Side.CENTER;
            Imgproc.rectangle(frame, CENTER_RECTANGLE, new Scalar(255, 255, 255));
        } else {
            location = Side.RIGHT;
            Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(255, 255, 255));
        }

        Imgproc.rectangle(finalMat, LEFT_RECTANGLE, new Scalar(255, 255, 255));
        Imgproc.rectangle(finalMat, CENTER_RECTANGLE, new Scalar(255, 255, 255));

        finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return null;            //You do not return the original mat anymore, instead return null

    }

    /*private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }*/

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        /*Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(LEFT_RECTANGLE, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(CENTER_RECTANGLE, scaleBmpPxToCanvasPx), rectPaint);*/
    }


    public Side getPropPosition() {
        return this.location;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}