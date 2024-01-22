package org.firstinspires.ftc.teamcode.auton.experimental;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.common.centerstage.LeftPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@Config
@Autonomous(name="ExperimentalAudienceRedAuto", group="Auto")
public class ExperimentalAudienceRedAuto extends LinearOpMode {
    private LeftPropPipeline redPropThreshold;
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
        Globals.COLOR = Side.RED;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        redPropThreshold = new LeftPropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
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
                .lineToConstantHeading(new Vector2d(-34, -24))
                .back(10)
                .strafeLeft(15)
                .forward(25)
                .turn(Math.toRadians(-95))
                .forward(50)
                .turn(Math.toRadians(-5))
                .forward(30)
                .turn(Math.toRadians(10))
                .lineToConstantHeading(new Vector2d(59, -38))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-48, -33))
                .back(15)
                .turn(Math.toRadians(-90))
                .back(10)
                .strafeLeft(38)
                .forward(50)
                .turn(Math.toRadians(-5))
                .forward(30)
                .turn(Math.toRadians(5))
                .lineToConstantHeading(new Vector2d(59, -27))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(29)
                .turn(Math.toRadians(-90))
                .forward(11)
                .back(13)
                .strafeLeft(25)
                .turn(Math.toRadians(-5))
                .forward(40)
                .turn(Math.toRadians(-5))
                .forward(30)
                .turn(Math.toRadians(10))
                .lineToConstantHeading(new Vector2d(59, -39))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .waitSeconds(1)
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
}
