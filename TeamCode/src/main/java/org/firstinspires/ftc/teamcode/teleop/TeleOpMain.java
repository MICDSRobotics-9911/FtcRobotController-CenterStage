package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
@Config
@TeleOp(name="MainTeleOp", group="OpMode")
public class TeleOpMain extends OpMode {

    private RobotHardware robot;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private double loopTime = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    private double speedModifier = 1;

    @Override
    public void init() {
        Globals.IS_USING_IMU = false;
        Globals.IS_AUTO = false;
        Globals.USING_DASHBOARD = true;
        robot = RobotHardware.getInstance();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, this.telemetry);
        // robot.addSubsystem(extension, intake);
        robot.read();
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }
    @Override
    public void start() {
        robot.airplaneHold.setPosition(0);
        runtime.reset();
    }

    @Override
    public void loop() {
        robot.read();
        if (gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            speedModifier = 0.3;
        } else {
            speedModifier = 1;
        }
        robot.drivetrain.driveRobotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), speedModifier);
        telemetry.addLine(robot.drivetrain.toString());
        robot.periodic();
        if (gamepad1.a || gamepad2.a) {
            robot.airplaneHold.setPosition(0);
        }
        if (gamepad1.x || gamepad2.x) {
            robot.airplaneHold.setPosition(1);
            double timer = runtime.seconds();
            sleep(1000);
            robot.airplaneLaunch.setPosition(1);
            sleep(1000);
            robot.airplaneLaunch.setPosition(0);
        }
        if (gamepad1.y || gamepad2.y) {
            robot.airplaneHold.setPosition(1);
            robot.airplaneLaunch.setPosition(0);
        }
        if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_DOWN) || gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            robot.spoolHangMotor.setPower(1);
        } else if (gamepadEx2.isDown(GamepadKeys.Button.DPAD_UP) || gamepadEx.isDown(GamepadKeys.Button.DPAD_UP)) {
            robot.spoolHangMotor.setPower(-1);
        } else {
            robot.spoolHangMotor.setPower(0);
        }
        if (gamepad2.dpad_left || gamepad1.dpad_left) {
            robot.hangHolder.setPower(0.8);
        }  else if (gamepad2.dpad_right || gamepad1.dpad_right) {
            robot.hangHolder.setPower(-0.1);
        } else {
            robot.hangHolder.setPower(0);
        }
        if (gamepad1.right_bumper) {
            robot.server.setPosition(1);
        }
        if (gamepad1.right_stick_button) {
            robot.server.setPosition(0);
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
    public void stop() {
        robot.airplaneLaunch.setPosition(0);
    }
    private void sleep(double milliseconds) {
        double currentTime = runtime.milliseconds() + milliseconds;
        while (runtime.milliseconds() <= currentTime) {
        }
    }
}
