package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.FtcDashboardProcessor;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;


@Autonomous(name="BlueAuto", group="Auto")
public class BlueAuto extends LinearOpMode {
    private PropPipeline bluePropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private String output = "";
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        // robot.init(hardwareMap, telemetry);
        // robot.enabled = true;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        bluePropThreshold = new PropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
        while (!isStarted()) {
            telemetry.addData("camera: ", portal.getCameraState());
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        dashboard.startCameraStream(bluePropThreshold, 30);
        waitForStart();

        while (opModeIsActive()) {
            Side side = bluePropThreshold.getPropPosition();
            switch (side) {
                case LEFT:
                    output = "left";
                    break;
                case CENTER:
                    output = "center";
                    break;
                case RIGHT:
                    output = "right";
                    break;
                default:
            }
            telemetry.addData("Prop Position: ", output);
            telemetry.update();
            sleep(100L);
        }
        portal.close();
    }
}
