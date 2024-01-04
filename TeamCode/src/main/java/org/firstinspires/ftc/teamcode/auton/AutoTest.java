package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto-Encoder-Test", group="Auto")
public class AutoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // TODO: Encoder Ticks needs to be tuned for driving forward to spike mark


    // TODO: This tolerance also needs to be empirically tuned
    private int tolerance = 5;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;



    @Override
    public void runOpMode() throws InterruptedException {
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int target1 = 2000;
        int target2 = 2000;
        int target3 = 2000;
        int target4 = 2000;
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotor.setTargetPosition(target1);
        backRightMotor.setTargetPosition(target2);
        frontLeftMotor.setTargetPosition(target3);
        frontRightMotor.setTargetPosition(target4);

        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("backLeftPos: ", backLeftMotor.getCurrentPosition());
            telemetry.addData("backrightPos: ", backRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftPos: ", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightPos: ", frontRightMotor.getCurrentPosition());

            while (backLeftMotor.isBusy() || backRightMotor.isBusy() || frontLeftMotor.isBusy() || frontRightMotor.isBusy()) {
                backLeftMotor.setPower(0.3);
                frontLeftMotor.setPower(0.3);
                backRightMotor.setPower(0.3);
                frontRightMotor.setPower(0.3);
                telemetry.addData("backLeftPos: ", backLeftMotor.getCurrentPosition());
                telemetry.addData("backrightPos: ", backRightMotor.getCurrentPosition());
                telemetry.addData("frontLeftPos: ", frontLeftMotor.getCurrentPosition());
                telemetry.addData("frontRightPos: ", frontRightMotor.getCurrentPosition());
                telemetry.update();
            }
            telemetry.update();
        }
    }

    public void driveRobotCentric(double x, double y, double turn, double speedModifier) {
        double power, theta, backLeftPower, backRightPower, frontLeftPower, frontRightPower;
        if (speedModifier < 0) {
            speedModifier *= -1;
        }
        x *= speedModifier;
        y *= speedModifier;
        turn *= speedModifier;
        power = Math.hypot(x, y);
        theta = Math.atan2(y, x);


        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        backLeftPower = power * sin/max + turn;
        backRightPower = power * cos/max - turn;
        frontLeftPower = power * cos/max + turn;
        frontRightPower = power * sin/max - turn;

        if ((power + Math.abs(turn)) > 1) {
            frontLeftPower /= power + Math.abs(turn);
            frontRightPower /= power + Math.abs(turn);
            backLeftPower /= power + Math.abs(turn);
            backRightPower /= power + Math.abs(turn);
        }

        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
    }
}