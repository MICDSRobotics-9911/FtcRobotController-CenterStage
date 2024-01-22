package org.firstinspires.ftc.teamcode.auton.experimental;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name="HardCodeBackdropRedAuto", group="Auto")
public class HardCodeBackdropRedAuto extends LinearOpMode {
    private LeftPropPipeline redPropThreshold;
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
        Side location;
        while (!isStarted()) {
            location = redPropThreshold.getPropPosition();
            telemetry.addLine("auto in init");
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        FtcDashboard.getInstance().startCameraStream(redPropThreshold, 30);
        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(100));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-10))
                .forward(35)
                .back(35)
                .turn(Math.toRadians(-90))
                .forward(40)
                .strafeLeft(25)
                .addDisplacementMarker(() -> {
                    // Drop Yellow on Backdrop
                    robot.server.setPosition(1);
                })
                .strafeRight(25)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // Reset Yellow dropper
                    robot.server.setPosition(0);
                })
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-10))
                .strafeRight(11)
                .forward(30)
                .back(30)
                .turn(Math.toRadians(-90))
                .forward(30)
                .strafeLeft(15)
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .strafeRight(15)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // Reset yellow pixel dropper
                    robot.server.setPosition(0);
                })
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-10))
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(10)
                .back(20)
                .turn(Math.toRadians(180))
                .forward(32)
                .strafeLeft(2)
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel on backdrop
                    robot.server.setPosition(1);
                })
                .strafeRight(30)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // Reset Yellow Pixel on backdrop
                    robot.server.setPosition(0);
                })
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
