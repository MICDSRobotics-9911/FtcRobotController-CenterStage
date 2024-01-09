package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
@Config
@TeleOp(name="MainTeleOp", group="LinearOpMode")
public class TeleOpMain extends LinearOpMode {

    private RobotHardware robot;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private double loopTime = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    private double speedModifier = 1;
    public static double launchPosition = 1;
    public static double hangPosition = 1;
    public static double secondHangPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_USING_IMU = false;
        Globals.IS_AUTO = false;
        Globals.USING_DASHBOARD = false;

        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        // robot.addSubsystem(extension, intake);
        robot.read();
        telemetry.addLine("Robot Initialized");
        telemetry.update();
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        // Manual reset
        robot.airplaneHold.setPosition(1);
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            robot.read();
            //robot.drivetrain.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), speedModifier);
            if (gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                speedModifier = 0.3;
            } else {
                speedModifier = 1;
            }
            robot.drivetrain.driveRobotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), speedModifier);
            telemetry.addLine(robot.drivetrain.toString());
            robot.periodic();
            if (gamepadEx.getButton(GamepadKeys.Button.A) || gamepadEx2.getButton(GamepadKeys.Button.A)) {
                robot.airplaneHold.setPosition(0);
                telemetry.addData("Servo Pos: ", robot.airplaneHold.getPosition());
            }
            if (gamepadEx.getButton(GamepadKeys.Button.B) || gamepadEx2.getButton(GamepadKeys.Button.B)) {
                robot.airplaneLaunch.setPosition(1);
            }
            if (gamepadEx.getButton(GamepadKeys.Button.Y) || gamepadEx2.getButton(GamepadKeys.Button.Y)) {
                robot.airplaneLaunch.setPosition(0);
            }
            if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepadEx.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                robot.hangHolder.setPower(0.3);
            } else if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_RIGHT) || gamepadEx.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                robot.hangHolder.setPower(0.3);
            } else {
                robot.hangHolder.setPower(0);
            }
            if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                robot.spoolHangMotor.setPower(1);
            } else if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_UP) || gamepadEx.isDown(GamepadKeys.Button.DPAD_UP)) {
                robot.spoolHangMotor.setPower(-1);
            } else {
                robot.spoolHangMotor.setPower(0);
            }
            /*if (gamepad2.a) {
                robot.hangHolder.setPower(0.3);
            } else if (gamepad2.x) {
                robot.hangHolder.setPower(0.3);
            } else {
                robot.hangHolder.setPower(0);
            }
            if (gamepad2.b) {
                robot.spoolHangMotor.setPower(1);
            } else if (gamepad2.y) {
                robot.spoolHangMotor.setPower(-1);
            } else {
                robot.spoolHangMotor.setPower(0);
            }*/
            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("Runtime: ", runtime.toString());
            telemetry.addData("Heading: ", robot.getAngle());
            loopTime = loop;
            robot.write();
            telemetry.update();
            robot.clearBulkCache();
        }
        if (isStopRequested()) {
            robot.airplaneLaunch.setPosition(0);
        }
    }
}
