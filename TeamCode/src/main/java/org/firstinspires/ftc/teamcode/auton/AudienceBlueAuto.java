package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name="AudienceBlueAuto", group="Auto")
public class AudienceBlueAuto extends LinearOpMode {
    private PropPipeline bluePropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private ElapsedTime runtime = new ElapsedTime();

    // TODO: Encoder Ticks needs to be tuned for driving forward to spike mark
    private int backLeftTarget = 0;
    private int backRightTarget = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;


    // TODO: This tolerance also needs to be empirically tuned
    private int tolerance = 5;
    private RobotHardware robot;
    private SampleMecanumDrive drive;
    public static int cases = 1;

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
        /*FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());*/
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
        //dashboard.startCameraStream(bluePropThreshold, 30);
        Pose2d startPose = new Pose2d(-34, 60, Math.toRadians(-100));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                // turn right
                .forward(11)
                .back(10)
                /*.lineToLinearHeading(new Pose2d(-34, 28, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(-34, 37))
                .turn(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60.25f, 35.41f), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(25)
                .forward(5)*/
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .strafeRight(3)
                .forward(7)
                /*.lineToLinearHeading(new Pose2d(-46, 35, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(-46, 41))
                .turn(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60.25f, 29.41f), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(22)
                .forward(5)*/
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .forward(7)
                .strafeLeft(3)
                .strafeRight(0.5)
                /*.lineToLinearHeading(new Pose2d(-34, 34, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-28, 34))
                .lineToConstantHeading(new Vector2d(-32, 34))
                .lineToConstantHeading(new Vector2d(-32, 44))
                .splineToConstantHeading(new Vector2d(60.25f, 41.41f), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(25)
                .forward(5)*/
                .build();
        waitForStart();
        location = bluePropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
        if (!isStopRequested() && opModeIsActive()) {
            location = bluePropThreshold.getPropPosition();
            /*switch (cases) {
                case 1:
                    location = Side.CENTER;
                    break;
                case 2:
                    location = Side.LEFT;
                    break;
                case 3:
                    location = Side.RIGHT;
            }*/
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
        robot.read();
        robot.periodic();
        robot.write();
        robot.clearBulkCache();
        runtime.reset();
        portal.close();
    }
}
