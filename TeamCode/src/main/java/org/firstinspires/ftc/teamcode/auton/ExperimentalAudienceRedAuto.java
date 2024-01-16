package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
@Config
@Autonomous(name="ExperimentalAudienceRedAuto", group="Auto")
public class ExperimentalAudienceRedAuto extends LinearOpMode {
    private PropPipeline redPropThreshold;
    private VisionPortal portal;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot;
    private SampleMecanumDrive drive;
    public static double TURN_VALUE = -3;
    public static double DISTANCE = 10; // in
    public static double SECOND_DISTANCE = 0.4;
    public static int cases = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.RED;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        redPropThreshold = new PropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        Side location;
        while (!isStarted()) {
            location = redPropThreshold.getPropPosition();
            telemetry.addLine("auto in init");
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        FtcDashboard.getInstance().startCameraStream(redPropThreshold, 30);
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34, -28, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-34, -35))
                .strafeLeft(15)
                .forward(25)
                .turn(Math.toRadians(-90))
                .forward(60)
                .lineToConstantHeading(new Vector2d(60.25f, -35.41f))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeLeft(20)
                .forward(5)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-46, -33))
                .back(15)
                .lineToLinearHeading(new Pose2d(-34, -35, Math.toRadians(0)))
                .strafeLeft(22)
                .forward(60)
                .lineToConstantHeading(new Vector2d(60.25f, -29.41f))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeLeft(20)
                .forward(5)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-30, -34))
                .lineToLinearHeading(new Pose2d(-40, -34, Math.toRadians(0)))
                .strafeLeft(23)
                .forward(60)
                .lineToConstantHeading(new Vector2d(60.25f, -41.41f))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeLeft(28)
                .forward(5)
                .build();
        waitForStart();
        location = redPropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
        if (!isStopRequested() && opModeIsActive()) {
            location = redPropThreshold.getPropPosition();
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
        if (isStopRequested()) {
            robot.read();
            robot.periodic();
            robot.write();
            robot.clearBulkCache();
            portal.close();
        }
    }
}
