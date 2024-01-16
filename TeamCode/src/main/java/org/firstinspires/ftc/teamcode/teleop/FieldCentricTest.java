package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
@Config
@TeleOp(name="field-centric", group="LinearOpMode")
public class FieldCentricTest extends LinearOpMode {

    private RobotHardware robot;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private double loopTime = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    private double speedModifier = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_USING_IMU = true;
        Globals.IS_AUTO = false;
        Globals.USING_DASHBOARD = true;

        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        // robot.addSubsystem(extension, intake);
        robot.read();
        telemetry.addLine("Robot Initialized");
        telemetry.update();
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        // Manual reset
        waitForStart();
        robot.airplaneHold.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            robot.read();
            //robot.drivetrain.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), speedModifier);
            if (gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                speedModifier = 0.3;
            } else {
                speedModifier = 1;
            }
            robot.drivetrain.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), speedModifier);
            telemetry.addLine(robot.drivetrain.toString());
            robot.periodic();
            if (gamepad1.a || gamepad2.a) {
                robot.airplaneHold.setPosition(0);
            }
            if (gamepad1.x || gamepad2.x) {
                robot.airplaneHold.setPosition(1);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.airplaneLaunch.setPosition(1);
                sleep(2000);
                robot.airplaneLaunch.setPosition(0);
            }
            if (gamepad1.y || gamepad2.y) {
                robot.airplaneLaunch.setPosition(0);
                robot.airplaneLaunch.setPosition(0);
            }
            if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                robot.spoolHangMotor.setPower(1);
            } else if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_UP) || gamepadEx.isDown(GamepadKeys.Button.DPAD_UP)) {
                robot.spoolHangMotor.setPower(-1);
            } else {
                robot.spoolHangMotor.setPower(0);
            }
            if (gamepad2.dpad_right || gamepad1.dpad_right) {
                robot.hangHolder.setPower(0.7);

            } else if (gamepad2.dpad_left || gamepad1.dpad_left) {
                robot.hangHolder.setPower(-0.7);
            } else {
                robot.hangHolder.setPower(0);
            }

            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("Runtime: ", runtime.toString());
            telemetry.addData("Heading: ", robot.getAngle());
            loopTime = loop;
            robot.write();
            telemetry.update();
            robot.clearBulkCache();
        }
    }
}
