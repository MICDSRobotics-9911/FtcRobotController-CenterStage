package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name="CameraTest", group="Auto")
public class CameraStreamTest extends LinearOpMode {
    private FtcDashboardProcessor dashboardProcessor;
    private VisionPortal portal;
    private static int CAMERA_WIDTH = 640;
    private static int CAMERA_HEIGHT = 480;
    private String output = "";
    private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() throws InterruptedException {
            Globals.IS_AUTO = true;
            Globals.IS_USING_IMU = false;
            Globals.USING_DASHBOARD = true;
            Globals.COLOR = Side.RED;
            WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

            // robot.init(hardwareMap, telemetry);
            // robot.enabled = true;
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            dashboardProcessor = new FtcDashboardProcessor(telemetry);
            portal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(dashboardProcessor)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();
            //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
            while (!isStarted()) {
                /*telemetry.addLine("camera test in init");
                telemetry.addData("camera: ", portal.getCameraState());
                telemetry.update();*/
            }
            dashboard.startCameraStream(dashboardProcessor, 30);
            waitForStart();

            while (opModeIsActive()) {
                sleep(100L);
            }
            portal.close();
        }
    }
