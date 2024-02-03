package org.firstinspires.ftc.teamcode.auton.experimental;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@Autonomous(name="CameraTest", group="Auto")
public class CameraStreamTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    private static int CAMERA_WIDTH = 640;
    private static int CAMERA_HEIGHT = 480;

        @Override
        public void runOpMode() throws InterruptedException {
            Globals.IS_AUTO = true;
            Globals.IS_USING_IMU = true;
            Globals.USING_DASHBOARD = true;
            Globals.COLOR = Side.RED;
            WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

            aprilTag = new AprilTagProcessor.Builder().build();

            portal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(aprilTag)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();

            while (!isStarted()) {
            }
            waitForStart();

            while (opModeIsActive()) {
                sleep(100L);
            }
            portal.close();
        }
    }
