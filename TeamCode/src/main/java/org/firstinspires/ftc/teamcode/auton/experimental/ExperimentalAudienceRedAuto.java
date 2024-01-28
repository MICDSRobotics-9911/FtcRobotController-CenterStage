package org.firstinspires.ftc.teamcode.auton.experimental;

import static org.firstinspires.ftc.teamcode.common.centerstage.HelpfulAprilTagDetection.getCameraSetting;
import static org.firstinspires.ftc.teamcode.common.centerstage.HelpfulAprilTagDetection.setManualExposure;
import static org.firstinspires.ftc.teamcode.common.util.MathUtils.getFCPosition;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.centerstage.LeftPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.common.centerstage.HelpfulAprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
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

        Pose2d startPose = new Pose2d(-38, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -20, Math.toRadians(0)))
                .back(10)
                .strafeLeft(10)
                .forward(60)
                .addDisplacementMarker(() -> {
                    drive.setPoseEstimate(getFCPosition(aprilTagProcessor.getFreshDetections(), drive.getRawExternalHeading()));
                    drive.update();
                })
                .forward(0.1)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(54, -31, Math.toRadians(0)))
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
                .lineToLinearHeading(new Pose2d(-48, -33, Math.toRadians(0)))
                .back(5)
                .strafeLeft(20)
                .forward(50)
                .lineToConstantHeading(new Vector2d(52, -18))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .back(5)
                .strafeLeft(5)
                .forward(12)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(0)))
                .forward(10)
                .back(10)
                .strafeLeft(20)
                .forward(50)
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
                .forward(13)
                .build();
        getCameraSetting(portal);
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;
        setManualExposure(portal, myExposure, myGain);
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
}
