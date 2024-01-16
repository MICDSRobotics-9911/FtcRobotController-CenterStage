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
@Autonomous(name="BackdropRedAuto", group="Auto")
public class BackdropRedAuto extends LinearOpMode {
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
            robot.log("auto in init");
            robot.log("camera: ", portal.getCameraState());
            robot.log("Prop Location: ", location.toString());
            telemetry.update();
        }
        FtcDashboard.getInstance().startCameraStream(redPropThreshold, 30);
        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(100));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .forward(10)
                .back(10)
                .turn(Math.toRadians(-20))
                .forward(10)
                .strafeLeft(3)
                .forward(2)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(1);
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(20)
                .forward(5)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .strafeRight(2)
                .forward(7)
                .back(7)
                .turn(Math.toRadians(-20))
                .forward(7)
                .strafeLeft(2)
                .forward(2)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(1);
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(20)
                .forward(5)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .forward(7)
                .strafeLeft(3)
                .strafeRight(0.5)
                .build();
        waitForStart();
        location = redPropThreshold.getPropPosition();
        robot.log("Prop Location: ", location.toString());
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
            robot.log("Prop Location: ", location.toString());
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
