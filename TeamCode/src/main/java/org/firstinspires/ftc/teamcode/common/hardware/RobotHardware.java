package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
public class RobotHardware {
    private static double p = 0.0, i = 0.0, d = 0.0;
    private double errorTolerance = 0.0;
    private int drivePosTolerance = 50;
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx extensionMotor;
    public DcMotorEx intakeMotor;
    public ServoEx airplaneLaunch;
    public ServoEx airplaneHold;
    public ServoEx clawServo;
    public WEncoder extensionEncoder;
    public WActuatorGroup extensionActuator;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private static RobotHardware instance = null;
    public boolean enabled;
    public List<LynxModule> modules;
    private ArrayList<WSubsystem> subsystems;
    IMU imu;


    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        // Add an if else detecting whether dashboard is running for telemetry
        this.telemetry = telemetry;


        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        this.subsystems = new ArrayList<>();

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // DRIVETRAIN
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Globals.IS_AUTO) {
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu = hardwareMap.get(IMU.class, "imu 1");
        imu.initialize(parameters);
        imu.resetYaw();

        // Extension
        /*extensionMotor = hardwareMap.get(DcMotorEx.class, "extension_motor");
        extensionEncoder = new WEncoder(new MotorEx(hardwareMap, "front_left_drive").encoder);
        this.extensionActuator = new WActuatorGroup(extensionMotor, extensionEncoder)
                .setPIDController(new PIDController(p, i, d))
                .setErrorTolerance(errorTolerance);*/

        // Intake
        //intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        // Outtake
        //clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);

        // Airplane Launch
        //airplaneLaunch = new SimpleServo(hardwareMap, "airplane_launch", 0, 270);
        //airplaneHold = new SimpleServo(hardwareMap, "airplane_hold", 0, 270);

        // Hang

    }

    public void read() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
        for (WSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void driveFieldCentric(double x, double y, double rx) {
        double power, theta, botHeading, rotX, rotY;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 0.8; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        setDrivePowers(backLeftPower, backRightPower, frontLeftPower, frontRightPower);
        telemetry.addData("backLeft: ", backLeftPower);
        telemetry.addData("backRight: ", backRightPower);
        telemetry.addData("frontLeft: ", frontLeftPower);
        telemetry.addData("frontRight: ", frontRightPower);
        telemetry.addData("robotHeading: ", botHeading);
        telemetry.update();
    }

    public void setDrivePowers(double backLeft, double backRight, double frontLeft, double frontRight) {
        this.backLeftMotor.setPower(backLeft);
        this.backRightMotor.setPower(backRight);
        this.frontLeftMotor.setPower(frontLeft);
        this.frontRightMotor.setPower(frontRight);
    }

    public void driveRobotCentric(double x, double y, double turn, double speedModifier) {
        double power, theta, backLeftPower, backRightPower, frontLeftPower, frontRightPower;
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
        setDrivePowers(backLeftPower * speedModifier, backRightPower * speedModifier, frontLeftPower * speedModifier, frontRightPower * speedModifier);
        telemetry.addData("backLeft: ", backLeftPower);
        telemetry.addData("backRight: ", backRightPower);
        telemetry.addData("frontLeft: ", frontLeftPower);
        telemetry.addData("frontRight", frontRightPower);
        telemetry.update();
    }

    public int setDriveTrainTarget(int target) {
        backLeftMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        frontLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);

        backLeftMotor.setTargetPositionTolerance(drivePosTolerance);
        backRightMotor.setTargetPositionTolerance(drivePosTolerance);
        frontLeftMotor.setTargetPositionTolerance(drivePosTolerance);
        frontRightMotor.setTargetPositionTolerance(drivePosTolerance);
        return drivePosTolerance;
    }

    public void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

    public double getVoltage() {
        return voltage;
    }

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }
}
