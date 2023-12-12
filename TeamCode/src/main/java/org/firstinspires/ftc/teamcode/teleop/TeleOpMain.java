package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name="MainTeleOp", group="LinearOpMode")
public class TeleOpMain extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.init(hardwareMap, telemetry);
        // robot.addSubsystem(extension, intake);
        robot.read();
        telemetry.addLine("Robot Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.read();
            robot.robotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX());

            robot.periodic();
            robot.write();
            robot.clearBulkCache();
        }
    }
}
