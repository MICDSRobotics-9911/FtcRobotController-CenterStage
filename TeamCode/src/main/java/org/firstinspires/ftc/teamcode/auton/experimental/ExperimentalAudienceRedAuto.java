package org.firstinspires.ftc.teamcode.auton.experimental;

import static org.firstinspires.ftc.teamcode.common.centerstage.HelpfulAprilTagDetection.setManualExposure;
import static org.firstinspires.ftc.teamcode.common.util.MathUtils.getFCPosition;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.centerstage.LeftPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Disabled
@Config
@Autonomous(name="ExperimentalAudienceRedAuto", group="Auto")
public class ExperimentalAudienceRedAuto extends LinearOpMode {
    private int myExposure;
    private int minExposure;
    private int maxExposure;
    private int myGain;
    private int minGain;
    private int maxGain;
    private LeftPropPipeline redPropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private RobotHardware robot;
    private SampleMecanumDrive drive;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.RED;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();
        redPropThreshold = new LeftPropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessors(redPropThreshold, aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        FtcDashboard.getInstance().startCameraStream(redPropThreshold, 30);
        Side location;
        while (!isStarted()) {
            location = redPropThreshold.getPropPosition();
            telemetry.addLine("auto in init");
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        Pose2d startPose = new Pose2d(-38, -61.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        drive.update();
        getCameraSetting();
        setManualExposure(portal, 6, 250);
        aprilTagProcessor.setDecimation(2);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -20, Math.toRadians(0)))
                .back(10)
                .strafeLeft(10)
                .forward(70)
                .lineToConstantHeading(new Vector2d(43, -34))
                .addDisplacementMarker(() -> {
                    List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getFreshDetections();
                    drive.setPoseEstimate(getFCPosition(aprilTagDetections, drive.getRawExternalHeading()));
                    drive.update();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(52, -31))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .back(3)
                .strafeLeft(20)
                .forward(11)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-44, -30, Math.toRadians(0)))
                .back(5)
                .strafeLeft(20)
                .forward(70)
                .lineToConstantHeading(new Vector2d(43, -34))
                .addDisplacementMarker(() -> {
                    List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getFreshDetections();
                    drive.setPoseEstimate(getFCPosition(aprilTagDetections, drive.getRawExternalHeading()));
                    drive.update();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(52, -26))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    // Reset yellow pixel
                    robot.server.setPosition(0);
                })
                .back(5)
                .strafeLeft(12)
                .forward(12)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, -33, Math.toRadians(0)))
                .forward(7)
                .back(10)
                .strafeLeft(20)
                .forward(70)
                .lineToConstantHeading(new Vector2d(43, -34))
                .addDisplacementMarker(() -> {
                    List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getFreshDetections();
                    drive.setPoseEstimate(getFCPosition(aprilTagDetections, drive.getRawExternalHeading()));
                    drive.update();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(52, -37))
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    // Reset Yellow Pixel
                    robot.server.setPosition(0);
                })
                .back(5)
                .strafeLeft(26)
                .forward(10)
                .build();
        waitForStart();
        if (!isStopRequested() && opModeIsActive()) {
            location = redPropThreshold.getPropPosition();
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
            switch (location) {
                case CENTER:
                    drive.followTrajectorySequence(centerTraj);
                    break;
                case LEFT:
                    drive.followTrajectorySequence(leftTraj);
                    break;
                default:
                    drive.followTrajectorySequence(rightTraj);
            }
        }

        if (isStopRequested()) {
            robot.read();
            robot.periodic();
            robot.write();
            robot.clearBulkCache();
            portal.close();
        }
    }
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (portal == null) {
            return;
        }

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = portal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }
}
