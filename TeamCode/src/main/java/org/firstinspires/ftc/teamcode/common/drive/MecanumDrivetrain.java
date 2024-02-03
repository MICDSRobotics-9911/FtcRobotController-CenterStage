package org.firstinspires.ftc.teamcode.common.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

@Config
public class MecanumDrivetrain extends WSubsystem {
    private final RobotHardware robot = RobotHardware.getInstance();

    double[] ws = new double[4];


    public MecanumDrivetrain() {

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
        double botHeading, rotX, rotY;
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

    public void setDrivetrainMode(DcMotorEx.RunMode mode) {
        robot.backLeftMotor.setMode(mode);
        robot.backRightMotor.setMode(mode);
        robot.frontLeftMotor.setMode(mode);
        robot.frontRightMotor.setMode(mode);
    }

    public boolean isBusy() {
        return robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy();
    }


    @Override
    public void periodic() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        setDrivePowers(ws[0], ws[1], ws[2], ws[3]);
    }

    @Override
    public void reset() {
        if (Globals.IS_USING_IMU) {
            robot.imu.resetYaw();
        }
        setDrivetrainMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stopDrive();
    }

    public String toString() {
        return "backLeftPower: " + robot.backLeftMotor.getPower() + "\nbackRightPower: " + robot.backRightMotor.getPower() + "\nfrontLeftPower: " + robot.frontLeftMotor.getPower() + "\nfrontRightPower: " + robot.frontRightMotor.getPower();
    }
}
