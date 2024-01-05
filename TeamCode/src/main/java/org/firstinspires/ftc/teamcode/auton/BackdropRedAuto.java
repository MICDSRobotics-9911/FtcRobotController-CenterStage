package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name="BackdropRedAuto", group="Auto")
public class BackdropRedAuto extends LinearOpMode {
    private PropPipeline redPropThreshold;
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
    public static double DISTANCE = 3; // in
    public static double SECOND_DISTANCE = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = false;
        Globals.COLOR = Side.RED;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        /*FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());*/
        redPropThreshold = new PropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        Side location = redPropThreshold.getPropPosition();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        //dashboard.startCameraStream(redPropThreshold, 30);
        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(32)
                /*.back(10)
                .turn(Math.toRadians(-90))
                .forward(40)
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(20)
                .forward(5)*/
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(11)
                /*.forward(32)
                .back(10)
                .turn(Math.toRadians(-90))
                .forward(28)
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(20)
                .forward(5)*/
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(29)
                /*.strafeLeft(11)
                .strafeRight(14)
                .turn(Math.toRadians(-90))
                .forward(35)
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                })
                .strafeRight(30)
                .forward(5)*/
                .build();
        waitForStart();
        location = redPropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
        if (!isStopRequested() && opModeIsActive()) {
            drive.followTrajectorySequence(centerTraj);
        }
        robot.read();
        robot.periodic();
        robot.write();
        robot.clearBulkCache();
        portal.close();
    }
}
