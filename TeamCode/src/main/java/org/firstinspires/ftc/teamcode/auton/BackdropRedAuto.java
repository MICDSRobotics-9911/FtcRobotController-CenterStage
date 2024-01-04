package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;

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

    @Override
    public void runOpMode() throws InterruptedException {
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
        waitForStart();
        robot.drivetrain.setDriveTrainTarget(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
        robot.drivetrain.setDriveToPosition(tolerance);
        while (opModeIsActive()) {
            location = redPropThreshold.getPropPosition();
            telemetry.addData("Prop Location: ", location.toString());
            while (robot.drivetrain.isBusy()) {
                robot.drivetrain.driveForward(0.5);
            }
            /*
            switch (location) {
                case LEFT:
                    // TODO: Tune strafe encoder values
                    backLeftTarget = 0;
                    backRightTarget = 0;
                    frontLeftTarget = 0;
                    frontRightTarget = 0;
                    robot.drivetrain.setDriveTrainTarget(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
                    robot.drivetrain.setDriveToPosition(tolerance);
                    // Strafe Left to the spike mark
                    while (robot.drivetrain.isBusy()) {
                        robot.drivetrain.strafeLeft(0.3);
                    }
                    // Set encoder values for the next trajectory of going forward to the backdrop
                    backLeftTarget = 0;
                    backRightTarget = 0;
                    frontLeftTarget = 0;
                    frontRightTarget = 0;
                    break;
                case CENTER:
                    // TODO: Tune forward encoder values
                    backLeftTarget = 0;
                    backRightTarget = 0;
                    frontLeftTarget = 0;
                    frontRightTarget = 0;
                    robot.drivetrain.setDriveTrainTarget(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
                    robot.drivetrain.setDriveToPosition(tolerance);
                    // Drive a little more forward to the spike mark
                    while (robot.drivetrain.isBusy()) {
                        robot.drivetrain.driveForward(0.3);
                    }
                    // Set encoder values for the next trajectory of going forward to the backdrop
                    backLeftTarget = 0;
                    backRightTarget = 0;
                    frontLeftTarget = 0;
                    frontRightTarget = 0;
                    break;
                default:
                    // TODO: Tune strafe encoder values
                    backLeftTarget = 0;
                    backRightTarget = 0;
                    frontLeftTarget = 0;
                    frontRightTarget = 0;
                    robot.drivetrain.setDriveTrainTarget(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
                    robot.drivetrain.setDriveToPosition(tolerance);
                    // Strafe Right to the spike mark
                    while (robot.drivetrain.isBusy()) {
                        robot.drivetrain.strafeRight(0.3);
                    }
                    // TODO: Tune forward encoder values
                    // Set encoder values for the next trajectory of going forward to the backdrop
                    backLeftTarget = 0;
                    backRightTarget = 0;
                    frontLeftTarget = 0;
                    frontRightTarget = 0;
                    break;
            }

            // TODO: Tune turn encoder values
            robot.drivetrain.setDriveTrainTarget(0, 0, 0, 0);
            robot.drivetrain.setDriveToPosition(tolerance);
            // Turning to face the backdrop
            while (robot.drivetrain.isBusy()) {
                robot.drivetrain.turnLeft(0.3);
            }

            // TODO: Tune forward encoder values
            robot.drivetrain.setDriveTrainTarget(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
            robot.drivetrain.setDriveToPosition(tolerance);
            // Drive to Backdrop
            while (robot.drivetrain.isBusy()) {
                robot.drivetrain.driveForward(0.5);
            }

            // Drop pixel based on randomization


             */
            robot.read();
            telemetry.addData("backLeftPos: ", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("backrightPos: ", robot.drivetrain.backRightPos);
            telemetry.addData("frontLeftPos: ", robot.drivetrain.frontLeftPos);
            telemetry.addData("frontRightPos: ", robot.drivetrain.frontRightPos);
            robot.periodic();
            robot.write();
            telemetry.update();
            robot.clearBulkCache();
        }
        portal.close();
    }
}
