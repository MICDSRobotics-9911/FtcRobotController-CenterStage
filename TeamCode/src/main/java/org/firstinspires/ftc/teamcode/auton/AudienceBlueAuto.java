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
    private RobotHardware robot;
    private SampleMecanumDrive drive;
    public static int cases = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;
        Globals.USING_DASHBOARD = true;
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
        Side location;
        while (!isStarted()) {
            location = bluePropThreshold.getPropPosition();
            robot.log("auto in init");
            robot.log("camera: ", portal.getCameraState());
            robot.log("Prop Location: ", location.toString());
            telemetry.update();
        }
        FtcDashboard.getInstance().startCameraStream(bluePropThreshold, 30);
        Pose2d startPose = new Pose2d(-34, 60, Math.toRadians(-100));
        drive.setPoseEstimate(startPose);
        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .forward(11)
                .back(10)
                .build();
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .strafeRight(3)
                .forward(7)
                .back(7)
                .build();
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-1))
                .forward(7)
                .strafeLeft(3)
                .strafeRight(0.5)
                .build();
        waitForStart();
        location = bluePropThreshold.getPropPosition();
        robot.log("Prop Location: ", location.toString());
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
        robot.read();
        robot.periodic();
        robot.write();
        robot.clearBulkCache();
        runtime.reset();
        portal.close();
    }
}
