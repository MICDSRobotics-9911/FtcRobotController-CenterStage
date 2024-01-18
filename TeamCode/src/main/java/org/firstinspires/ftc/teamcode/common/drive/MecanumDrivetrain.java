package org.firstinspires.ftc.teamcode.common.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;

@Config
public class MecanumDrivetrain extends WSubsystem {
    private final RobotHardware robot = RobotHardware.getInstance();

    double[] ws = new double[4];
    public double backLeftPos = 0.0;
    public double backRightPos = 0.0;
    public double frontLeftPos = 0.0;
    public double frontRightPos = 0.0;


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

        ws[0] = backLeftPower;
        ws[1] = backRightPower;
        ws[2] = frontLeftPower;
        ws[3] = frontRightPower;
        setDrivePowers(ws[0], ws[1], ws[2], ws[3]);
    }

    public void driveFieldCentric(double x, double y, double rx, double speedModifier) {
        double power, theta, botHeading, rotX, rotY;
        if (speedModifier < 0) {
            speedModifier *= -1;
        }
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

    public void setDrivetrainTarget(int backLeftTarget, int backRightTarget, int frontLeftTarget, int frontRightTarget) {
        robot.backLeftMotor.setTargetPosition(backLeftTarget);
        robot.backRightMotor.setTargetPosition(backRightTarget);
        robot.frontLeftMotor.setTargetPosition(frontLeftTarget);
        robot.frontRightMotor.setTargetPosition(frontRightTarget);
    }

    public void setDrivetrainMode(DcMotorEx.RunMode mode) {
        robot.backLeftMotor.setMode(mode);
        robot.backRightMotor.setMode(mode);
        robot.frontLeftMotor.setMode(mode);
        robot.frontRightMotor.setMode(mode);
    }

    public boolean isBusy() {
        return robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy();
    }

    public void driveForward(double speed, double inches) {
        encoderDrive(speed, inches, inches, inches, inches);
    }
    public void driveBackwards(double speed, double inches) {
        encoderDrive(speed, -inches, -inches, -inches, -inches);
    }
    public void strafeRight(double speed, double inches) {
        encoderDrive(speed, inches, -inches, -inches, inches);
    }

    public void strafeLeft(double speed, double inches) {
        encoderDrive(speed, -inches, inches, inches, -inches);
    }

    public void turnRight(double speed) {
        // TODO: NEED TO TUNE TURN VALUES
        encoderDrive(speed, 10, -10, 10, -10);
    }

    public void turnLeft(double speed) {
        encoderDrive(speed, -10, 10, -10, 10);
    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLFTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (DriveConstants.inchesToEncoderTicks(leftFrontInches));
        newRFTarget = robot.frontRightMotor.getCurrentPosition() + (int) (DriveConstants.inchesToEncoderTicks(rightFrontInches));
        newLBTarget = robot.backLeftMotor.getCurrentPosition() + (int) (DriveConstants.inchesToEncoderTicks(leftBackInches));
        newRBTarget = robot.backRightMotor.getCurrentPosition() + (int) (DriveConstants.inchesToEncoderTicks(rightBackInches));

        robot.drivetrain.setDrivetrainTarget(newLBTarget * 20, newRBTarget, newLFTarget * 20, newRFTarget);

        // Turn On RUN_TO_POSITION
        robot.drivetrain.setDrivetrainMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.drivetrain.setDrivePowers(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (isBusy()) {
        }

        // Stop all motion;
        robot.drivetrain.stopDrive();

        // Turn off RUN_TO_POSITION
        robot.drivetrain.setDrivetrainMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        setDrivetrainMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stopDrive();
    }

    public String toString() {
        return "backLeftPower: " + robot.backLeftMotor.getPower() + "\nbackRightPower: " + robot.backRightMotor.getPower() + "\nfrontLeftPower: " + robot.frontLeftMotor.getPower() + "\nfrontRightPower: " + robot.frontRightMotor.getPower();
    }
}
