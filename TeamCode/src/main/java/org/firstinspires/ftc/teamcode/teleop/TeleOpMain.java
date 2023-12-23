package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name="MainTeleOp", group="LinearOpMode")
public class TeleOpMain extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private double loopTime = 0.0;
    ElapsedTime runtime = new ElapsedTime();

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
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            robot.read();
            //robot.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX());
            robot.driveRobotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), 1);
            robot.periodic();
            /*if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
                robot.airplaneHold.setPosition(0.5);
                sleep(1000);
                robot.airplaneLaunch.setPosition(1);
            }
            if (gamepadEx.isDown(GamepadKeys.Button.B)) {

            }*/
            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("Runtime: ", runtime.toString());
            loopTime = loop;
            telemetry.update();


            robot.write();
            robot.clearBulkCache();
        }
    }
}
