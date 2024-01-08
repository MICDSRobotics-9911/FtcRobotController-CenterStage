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
        robot.airplaneHold.setPosition(0);
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
            robot.drivetrain.setDrivePowers();
            telemetry.addLine(robot.drivetrain.toString());
            robot.periodic();
            if (gamepad1.a) {
                robot.airplaneHold.setPosition(1);
                telemetry.addData("Servo Pos: ", robot.airplaneHold.getPosition());
            }
            if (gamepad1.b) {
                robot.airplaneLaunch.setPosition(1);
            }
            if (gamepad1.y) {
                robot.airplaneLaunch.setPosition(0);
            }
            if (gamepadEx.isDown(GamepadKeys.Button.B)) {
            }
            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("Runtime: ", runtime.toString());
            telemetry.addData("Heading: ", robot.getAngle());
            telemetry.addData("backLeftPos: ", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("backrightPos: ", robot.backRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftPos: ", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightPos: ", robot.frontRightMotor.getCurrentPosition());
            loopTime = loop;
            robot.write();
            telemetry.update();
            robot.clearBulkCache();
        }
    }
}
