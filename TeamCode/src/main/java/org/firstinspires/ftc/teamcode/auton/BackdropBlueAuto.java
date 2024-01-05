package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    public static double DISTANCE = 3; // in
    public static double SECOND_DISTANCE = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
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
        drive.setPoseEstimate(new Pose2d());
        Trajectory center = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
        Trajectory center1 = drive.trajectoryBuilder(center.end())
                .forward(DISTANCE)
                .build();

        //TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d()).forward(DISTANCE - 2).waitSeconds(2).strafeRight(0.3).waitSeconds(2).strafeLeft(0.3).turn(Math.toRadians(-90)).forward()
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            location = bluePropThreshold.getPropPosition();
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.addData("backLeftPos: ", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightPos: ", robot.backRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftPos: ", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightPos: ", robot.frontRightMotor.getCurrentPosition());
            drive.followTrajectory(center);
            //drive.turn(Math.toRadians(-90));
            /*switch (location) {
                case LEFT:
                    robot.drivetrain.strafeLeft(0.3, 10);
                    robot.drivetrain.driveForward(0.5, 10);
                    robot.drivetrain.turnLeft(0.3);
                    robot.drivetrain.driveForward(0.5, 10);

                    break;
                case CENTER:

                    // TODO: Tune forward encoder values
                    robot.drivetrain.driveForward(0.5, 10);
                    robot.drivetrain.turnLeft(0.3);
                    robot.drivetrain.driveForward(0.5, 10);
                    break;
                default:
                    // TODO: Tune strafe encoder values
                    robot.drivetrain.driveForward(0.5, 10);
                    robot.drivetrain.strafeRight(0.3, 10);
                    robot.drivetrain.turnLeft(0.3);
                    robot.drivetrain.driveForward(0.5, 10);
                    break;
            }

            // TODO: Tune turn encoder values
            // Drop pixel based on randomization


             */
            robot.read();
            robot.periodic();
            robot.write();
            telemetry.update();
            robot.clearBulkCache();
        }
        portal.close();
    }
}
