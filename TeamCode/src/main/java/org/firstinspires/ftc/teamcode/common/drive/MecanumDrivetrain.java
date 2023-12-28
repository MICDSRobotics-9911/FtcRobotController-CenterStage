package org.firstinspires.ftc.teamcode.common.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
@Config
public class MecanumDrivetrain extends WSubsystem {
    private final RobotHardware robot = RobotHardware.getInstance();
    // TODO: tune this using the robot
    private final double ticksPerInch = 0.0;

    double[] ws = new double[4];
    private double backLeftPos = 0.0;
    private double backRightPos = 0.0;
    private double frontLeftPos = 0.0;
    private double frontRightPos = 0.0;
    private PIDController controller;
    private Telemetry telemetry;
    public static double p = 0, i = 0, d = 0;


    public MecanumDrivetrain(Telemetry telemetry) {
        this.telemetry = telemetry;
        controller = new PIDController(p, i, d);
        controller.setTolerance(100);
    }


    public void setDrivePowers() {
        robot.backLeftMotor.setPower(ws[0]);
        robot.backRightMotor.setPower(ws[1]);
        robot.frontLeftMotor.setPower(ws[2]);
        robot.frontRightMotor.setPower(ws[3]);
    }
    public void setDrivePowers(double backLeft, double backRight, double frontLeft, double frontRight) {
        robot.backLeftMotor.setPower(backLeft);
        robot.backRightMotor.setPower(backRight);
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
    }

    public void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

    public void driveRobotCentric(double x, double y, double turn, double speedModifier) {
        double power, theta, backLeftPower, backRightPower, frontLeftPower, frontRightPower;
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

        ws[0] = backLeftPower;
        ws[1] = backRightPower;
        ws[2] = frontLeftPower;
        ws[3] = frontRightPower;
        setDrivePowers(ws[0], ws[1], ws[2], ws[3]);
    }

    public void driveFieldCentric(double x, double y, double rx, double speedModifier) {
        double power, theta, botHeading, rotX, rotY;
        x *= speedModifier;
        y *= speedModifier;
        rx *= speedModifier;
        botHeading = robot.getAngle();
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 0.8; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        ws[0] = backLeftPower;
        ws[1] = backRightPower;
        ws[2] = frontLeftPower;
        ws[3] = frontRightPower;
        setDrivePowers();
    }

    public void setDriveTrainTarget(int target, int drivePosTolerance) {
        robot.drivetrain.resetEncoders();
        robot.backLeftMotor.setTargetPosition(target);
        robot.backRightMotor.setTargetPosition(target);
        robot.frontLeftMotor.setTargetPosition(target);
        robot.frontRightMotor.setTargetPosition(target);

        robot.backLeftMotor.setTargetPositionTolerance(drivePosTolerance);
        robot.backRightMotor.setTargetPositionTolerance(drivePosTolerance);
        robot.frontLeftMotor.setTargetPositionTolerance(drivePosTolerance);
        robot.frontRightMotor.setTargetPositionTolerance(drivePosTolerance);

        robot.backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void driveStraightToTarget(int target, int current) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ws[0] = controller.calculate(backLeftPos, target);
        ws[1] = controller.calculate(backRightPos, target);
        ws[2] = controller.calculate(frontLeftPos, target);
        ws[3] = controller.calculate(frontRightPos, target);

        setDrivePowers();

        telemetry.addData("backLeftPos: ", backLeftPos);
        telemetry.addData("backrightPos: ", backRightPos);
        telemetry.addData("frontLeftPos: ", frontLeftPos);
        telemetry.addData("frontRightPos: ", frontRightPos);
    }


    public void driveForward(double speed) {
        driveRobotCentric(0, 1, 0, speed);
        telemetry.addData("backLeftPos: ", backLeftPos);
        telemetry.addData("backrightPos: ", backRightPos);
        telemetry.addData("frontLeftPos: ", frontLeftPos);
        telemetry.addData("frontRightPos: ", frontRightPos);
    }
    public void resetEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy() {
        return robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy();
    }
    @Override
    public void periodic() {

    }

    @Override
    public void read() {
        backLeftPos = robot.backLeftMotor.getCurrentPosition();
        backRightPos = robot.backRightMotor.getCurrentPosition();
        frontLeftPos = robot.frontLeftMotor.getCurrentPosition();
        frontRightPos = robot.frontRightMotor.getCurrentPosition();
    }

    @Override
    public void write() {
        setDrivePowers(ws[0], ws[1], ws[2], ws[3]);
    }

    @Override
    public void reset() {
        robot.imu.resetYaw();
        resetEncoders();
        stopDrive();
    }

    public String toString() {
        return "WS0: " + ws[0] + "WS1: " + ws[1] + "WS2: " + ws[2] + "WS3: " + ws[3];
    }

}
