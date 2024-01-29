package org.firstinspires.ftc.teamcode.common.centerstage;
import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;
@Config
public class LeftPropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private Mat testMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat finalMat = new Mat();
    public static double redThreshold = 0.21;
    public static double blueThreshold = 0.21;
    public static double threshold = 0;
    private Side location = Side.RIGHT;

    private final Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
    private final Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

    private final Scalar highHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
    private final Scalar highHSVRedUpper = new Scalar(180, 255, 255);

    private final Scalar lowHSVBlueLower = new Scalar(80, 100, 100);
    private final Scalar highHSVBlueUpper = new Scalar(140, 255, 255);

    public static Rect LEFT_RECTANGLE = new Rect(0, 80, 180, 200);
    public static Rect CENTER_RECTANGLE = new Rect(240, 30, 320, 180);
    private Telemetry telemetry;
    private int fieldColor = Imgproc.COLOR_RGB2HSV;
    public LeftPropPipeline(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (Globals.COLOR == Side.RED) {
            // RED
            threshold = redThreshold;
            fieldColor = Imgproc.COLOR_RGB2HSV;
        }
        else {
            // BLUE
            threshold = blueThreshold;
            //fieldColor = Imgproc.COLOR_RGB2HSV_FULL;
        }
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert Image from RGB to HSV
        Imgproc.cvtColor(frame, testMat, fieldColor);

        if (Globals.COLOR == Side.RED) {
            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);
            Core.bitwise_or(lowMat, highMat, finalMat);
        } else {
            Core.inRange(testMat, lowHSVBlueLower, highHSVBlueUpper, finalMat);
        }

        testMat.release();
        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedCenterBox = centerBox / CENTER_RECTANGLE.area() / 255; //Makes value [0,1]
        telemetry.addData("Left Box: ", averagedLeftBox);
        telemetry.addData("Right Box: ", averagedCenterBox);
        telemetry.update();
        if (averagedLeftBox > threshold) {        //Must Tune Threshold
            location = Side.LEFT;
            Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(0, 255, 0));
            Imgproc.rectangle(frame, CENTER_RECTANGLE, new Scalar(255, 255, 255));
        } else if (averagedCenterBox > threshold) {
            location = Side.CENTER;
            Imgproc.rectangle(frame, CENTER_RECTANGLE, new Scalar(0, 255, 0));
            Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(255, 255, 255));
        } else {
            location = Side.RIGHT;
            Imgproc.rectangle(frame, CENTER_RECTANGLE, new Scalar(255, 0, 0));
            Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(255, 0, 0));
        }

        // These lines are for tuning the rectangles
        //Imgproc.rectangle(finalMat, LEFT_RECTANGLE, new Scalar(255, 255, 255));
        //Imgproc.rectangle(finalMat, CENTER_RECTANGLE, new Scalar(255, 255, 255));

        /*finalMat.copyTo(frame); This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return null;            //You do not return the original mat anymore, instead return null

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    public Side getPropPosition() {
        return this.location;
    }

    @Override   
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}