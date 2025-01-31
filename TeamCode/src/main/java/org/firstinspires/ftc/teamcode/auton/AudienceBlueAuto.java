package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.common.centerstage.LeftPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.RightPropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="AudienceBlueAuto", group="Auto")
public class AudienceBlueAuto extends LinearOpMode {
    private RightPropPipeline bluePropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private ElapsedTime runtime = new ElapsedTime();
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
        bluePropThreshold = new RightPropPipeline(telemetry);
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
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        Pose2d startPose = new Pose2d(-38, 61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        drive.update();
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-40, 21, Math.toRadians(0)))
                .back(10)
                .strafeLeft(38)
                .waitSeconds(5)
                .forward(70)
                .strafeRight(20)
                .lineToConstantHeading(new Vector2d(56, 33),
                        SampleMecanumDrive.getVelocityConstraint(11, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(11))
                .lineToConstantHeading(new Vector2d(55, 35),
                        SampleMecanumDrive.getVelocityConstraint(11, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(11))
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .strafeRight(0.1)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-53.5, 28, Math.toRadians(0)))
                .back(3)
                .strafeLeft(31)
                .waitSeconds(5)
                .forward(70)
                .strafeRight(10)
                .lineToConstantHeading(new Vector2d(56, 28),
                        SampleMecanumDrive.getVelocityConstraint(11, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(11))
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel
                    robot.server.setPosition(1);
                })
                .forward(1)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .strafeRight(0.1)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, 32, Math.toRadians(0)))
                .forward(6.9)
                .back(10)
                .strafeLeft(28)
                .waitSeconds(5)
                .forward(70)
                .strafeRight(10)
                .lineToConstantHeading(new Vector2d(56, 40),
                        SampleMecanumDrive.getVelocityConstraint(11, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(11))
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel
                    robot.server.setPosition(1);
                })
                .forward(1)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .strafeRight(0.1)
                .build();
        waitForStart();
        location = bluePropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
        if (!isStopRequested() && opModeIsActive()) {
            if (portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                portal.stopLiveView();
            }
            location = bluePropThreshold.getPropPosition();
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
            switch (location) {
                case CENTER:
                    drive.followTrajectorySequence(centerTraj);
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(rightTraj);
                    break;
                default:
                    drive.followTrajectorySequence(leftTraj);
            }
        }
        robot.read();
        robot.periodic();
        robot.write();
        robot.clearBulkCache();
        runtime.reset();
        portal.close();
    }
}
