package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
@TeleOp(name="MainTeleOp", group="LinearOpMode")
public class TeleOpMain extends LinearOpMode {

    private RobotHardware robot;
    private double loopTime = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    private double speedModifier = 1;

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
        // Manual reset
        robot.airplaneHold.setPosition(1);
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            robot.read();
            if (gamepad1.left_bumper || gamepad1.left_trigger > 0) {
                speedModifier = 0.3;
            } else {
                speedModifier = 1;
            }
            robot.drivetrain.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedModifier);
            telemetry.addLine(robot.drivetrain.toString());
            robot.periodic();
            if (gamepad1.a || gamepad2.a) {
                robot.airplaneHold.setPosition(0);
            }
            if (gamepad1.x || gamepad2.x) {
                robot.airplaneHold.setPosition(1);
                sleep(1000);
                robot.airplaneLaunch.setPosition(1);
                sleep(1000);
                robot.airplaneLaunch.setPosition(0);
            }
            if (gamepad2.dpad_left || gamepad1.dpad_left) {
                robot.hangHolder.setPower(0.8);
            } else if (gamepad2.dpad_right || gamepad1.dpad_right) {
                robot.hangHolder.setPower(-0.1);
            } else {
                robot.hangHolder.setPower(0);
            }
            if (gamepad2.dpad_down || gamepad1.dpad_down) {
                robot.spoolHangMotor.setPower(1);
            } else if (gamepad2.dpad_up || gamepad1.dpad_up) {
                robot.spoolHangMotor.setPower(-1);
            } else {
                robot.spoolHangMotor.setPower(0);
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
        if (isStopRequested()) {
            robot.airplaneLaunch.setPosition(0);
        }
    }
}