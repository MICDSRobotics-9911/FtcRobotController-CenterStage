package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name="BackdropRedAuto", group="Auto")
public class BackdropRedAuto extends LinearOpMode {
    private PropPipeline redPropThreshold;
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
        redPropThreshold = new PropPipeline(telemetry);
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
        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(100));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13, -26, Math.toRadians(90)))
                .back(20)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(52, -31))
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .forward(1)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .back(10)
                .strafeRight(28)
                .forward(15)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(27, -35, Math.toRadians(90)))
                .back(10)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(52, -37))
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(1);
                })
                .forward(0.5)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.server.setPosition(0);
                })
                .back(10)
                .strafeRight(25)
                .forward(15)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-10))
                .forward(27)
                .turn(Math.toRadians(90))
                .forward(8)
                .back(20)
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(52, -18))
                .addDisplacementMarker(() -> {
                    //drive.resetHeadingPID(telemetry);
                    robot.server.setPosition(1);
                })
                .forward(0.7)
                .addDisplacementMarker(() -> {
                    // Reset Yellow Pixel on backdrop
                    robot.server.setPosition(0);
                })
                .waitSeconds(1)
                .back(10)
                .strafeRight(32)
                .forward(15)
                .build();
        waitForStart();
        location = redPropThreshold.getPropPosition();
        telemetry.addData("Prop Location: ", location.toString());
        telemetry.update();
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
        robot.read();
        robot.periodic();
        robot.write();
        robot.clearBulkCache();
        portal.close();
    }
}
