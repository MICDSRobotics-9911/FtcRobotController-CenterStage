package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
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
        robot.drivetrain.setDrivetrainMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setDrivetrainMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            location = redPropThreshold.getPropPosition();
            telemetry.addData("Prop Location: ", location.toString());
            telemetry.addData("backLeftPos: ", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("backrightPos: ", robot.backRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftPos: ", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightPos: ", robot.frontRightMotor.getCurrentPosition());
            robot.drivetrain.driveForward(0.5, 10);


            /*
            switch (location) {
                case LEFT:
                    // TODO: Tune strafe encoder values
                    robot.drivetrain.driveForward(0.5, 10);
                    robot.drivetrain.strafeLeft(0.3, 10);
                    robot.drivetrain.turnRight(0.3);
                    robot.drivetrain.driveForward(0.5, 10);

                    break;
                case CENTER:
                    // TODO: Tune forward encoder values
                    robot.drivetrain.driveForward(0.5, 10);
                    robot.drivetrain.turnRight(0.3);
                    robot.drivetrain.driveForward(0.5, 10);
                    break;
                default:
                    // TODO: Tune strafe encoder values
                    robot.drivetrain.strafeRight(0.3, 10);
                    robot.drivetrain.driveForward(0.5, 10);
                    robot.drivetrain.turnRight(0.3);
                    robot.drivetrain.driveForward(0.5, 10);
                    break;
            }

            // TODO: Tune turn encoder values
            // Drop pixel based on randomization


             */
            robot.read();
            telemetry.addData("backLeftPos: ", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("backrightPos: ", robot.backRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftPos: ", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightPos: ", robot.frontRightMotor.getCurrentPosition());
            robot.periodic();
            robot.write();
            telemetry.update();
            robot.clearBulkCache();
        }
        portal.close();
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
}
