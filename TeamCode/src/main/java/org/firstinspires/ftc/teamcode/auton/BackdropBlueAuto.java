package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="BackdropBlueAuto", group="Auto")
public class BackdropBlueAuto extends LinearOpMode {
    private PropPipeline bluePropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private ElapsedTime runtime = new ElapsedTime();

    // TODO: Encoder Ticks needs to be tuned for driving forward to spike mark
    private int backLeftTarget = 2000;
    private int backRightTarget = -2000;
    private int frontLeftTarget = 2000;
    private int frontRightTarget = -2000;


    // TODO: This tolerance also needs to be empirically tuned
    private int tolerance = 5;
    private RobotHardware robot;
    SampleMecanumDrive drive;
    public static double DISTANCE = 0.385; // in
    public static double SECOND_DISTANCE = 0.1;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = false;
        Globals.COLOR = Side.BLUE;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        bluePropThreshold = new PropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        Side location = bluePropThreshold.getPropPosition();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(13, 35))
                .back(20)
                .splineToLinearHeading(new Pose2d(60.25f, 35.41, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeLeft(30)
                .forward(5)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22.5, 35))
                .back(20)
                .splineToLinearHeading(new Pose2d(60.25f, 41.41f, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeLeft(15)
                .forward(5)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13, 34, Math.toRadians(180)))
                .lineTo(new Vector2d(1.5, 34))
                .back(20)
                .splineToLinearHeading(new Pose2d(60.25f, 29.41f, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeLeft(30)
                .forward(5)
                .build();
        waitForStart();
        location = bluePropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
        while (!isStopRequested() && opModeIsActive()) {
            //drive.followTrajectorySequence(leftTraj);
            location = bluePropThreshold.getPropPosition();
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        robot.read();
        robot.periodic();
        robot.write();
        robot.clearBulkCache();
        runtime.reset();
        portal.close();
    }
}
