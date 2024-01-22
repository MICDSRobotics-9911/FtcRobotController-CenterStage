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
@Autonomous(name="HardCodeBackdropBlueAuto", group="Auto")
public class HardCodeBackdropBlueAuto extends LinearOpMode {
    private LeftPropPipeline bluePropThreshold;
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
            telemetry.addData("camera: ", portal.getCameraState());
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.update();
        }
        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(32)
                .back(32)
                .turn(Math.toRadians(90))
                .forward(35)
                .strafeRight(25)
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .strafeLeft(25)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // Reset Yellow Pixel Holder
                    robot.server.setPosition(0);
                })
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(8)
                .forward(30)
                .back(30)
                .turn(Math.toRadians(90))
                .forward(27)
                .strafeRight(20)
                .addDisplacementMarker(() -> {
                    // Drop Yellow pixel on backboard
                    robot.server.setPosition(1);
                })
                .strafeLeft(20)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // Reset Yellow pixel Holder
                    robot.server.setPosition(0);
                })
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(27)
                .turn(Math.toRadians(-90))
                .forward(10)
                .back(10)
                .turn(Math.toRadians(180))
                .forward(35)
                .strafeRight(4)
                .addDisplacementMarker(() -> {
                    // Drop Yellow Pixel
                    robot.server.setPosition(1);
                })
                .strafeLeft(30)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // Reset Yellow Pixel Holder
                    robot.server.setPosition(0);
                })

                .build();
        waitForStart();
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
        if (isStopRequested()) {
            robot.read();
            robot.periodic();
            robot.write();
            robot.clearBulkCache();
            portal.close();
        }
    }
}
