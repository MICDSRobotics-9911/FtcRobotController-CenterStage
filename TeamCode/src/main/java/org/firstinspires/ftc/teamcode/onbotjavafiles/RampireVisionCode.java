package org.firstinspires.ftc.teamcode.onbotjavafiles;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.LeftPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@Autonomous(name="RampireVision", group="Auto")
public class RampireVisionCode extends LinearOpMode {
    private LeftPropPipeline bluePropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        bluePropThreshold = new LeftPropPipeline(telemetry);
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        Side location;
        while (!isStarted()) {
            location = bluePropThreshold.getPropPosition();
            telemetry.addLine("auto in init");
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
            if (isStopRequested()) {
                portal.close();
            }
        }
        waitForStart();
        location = bluePropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
        if (!isStopRequested() && opModeIsActive()) {
            location = bluePropThreshold.getPropPosition();
        }
        portal.close();
    }
}


