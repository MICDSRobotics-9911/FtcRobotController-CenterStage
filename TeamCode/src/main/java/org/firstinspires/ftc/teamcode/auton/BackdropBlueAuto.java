package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.common.centerstage.LeftPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="BackdropBlueAuto", group="Auto")
public class BackdropBlueAuto extends LinearOpMode {
    private LeftPropPipeline bluePropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private RobotHardware robot;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        bluePropThreshold = new LeftPropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        FtcDashboard.getInstance().startCameraStream(bluePropThreshold, 30);
        Side location;
        while (!isStarted()) {
            location = bluePropThreshold.getPropPosition();
            telemetry.addLine("auto in init");
            telemetry.addData("Camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location);
            telemetry.update();
        }
        Pose2d startPose = new Pose2d(14, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(13, 26))
                .back(20)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(52, 41))
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(1);
                    // Drop Yellow pixel on backboard
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .back(5)
                .strafeLeft(25)
                .forward(5)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(24, 35))
                .back(20)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(52, 41))
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
                .strafeLeft(23)
                .forward(5)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(27)
                .turn(Math.toRadians(-90))
                .forward(13)
                .back(10)
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(52, 31))
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel
                    robot.server.setPosition(1);
                })
                .forward(0.3)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    // Reset Yellow Pixel Holder
                    robot.server.setPosition(0);
                })
                .back(3)
                .strafeLeft(29)
                .forward(15)
                .build();
        waitForStart();
        if (!isStopRequested() && opModeIsActive()) {
            location = bluePropThreshold.getPropPosition();
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
