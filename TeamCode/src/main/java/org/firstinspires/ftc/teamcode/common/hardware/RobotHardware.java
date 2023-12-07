package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx extensionMotor;
    public DcMotorEx intakeMotor;
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
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();

        // Extension
        extensionMotor = hardwareMap.get(DcMotorEx.class, "extension_motor");
        extensionEncoder = new WEncoder(new MotorEx(hardwareMap, "front_left_drive").encoder);
        this.extensionActuator = new WActuatorGroup(extensionMotor, extensionEncoder)
                .setPIDController(new PIDController(p, i, d))
                .setErrorTolerance(errorTolerance);

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        // Outtake
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);

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

    public void fieldCentricDrive(Gamepad gamepad1) {
        double x, y, power, rx, theta, botHeading, rotX, rotY, speedModifier;

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        if (gamepad1.options) {
            imu.resetYaw();
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 0.8; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // This speed modifier is for true racers
        speedModifier = 0.8 + (0.8 * gamepad1.right_trigger) - (.4 * gamepad1.left_trigger);

        this.backLeftMotor.setPower(backLeftPower);
        this.frontLeftMotor.setPower(frontLeftPower);
        this.frontRightMotor.setPower(frontRightPower);
        this.backRightMotor.setPower(backRightPower);
        telemetry.addData("backLeft: ", backLeftPower);
        telemetry.addData("backRight: ", backRightPower);
        telemetry.addData("frontLeft: ", frontLeftPower);
        telemetry.addData("frontRight: ", frontRightPower);
        telemetry.addData("robotHeading: ", botHeading);
        telemetry.update();
    }

    public void robotCentric(Gamepad gamepad1) {
        double x, y, power, turn, theta, backLeftPower, backRightPower, frontLeftPower, frontRightPower;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        power = Math.hypot(x, y);
        turn = gamepad1.right_stick_x;
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
        this.backLeftMotor.setPower(backLeftPower);
        this.frontLeftMotor.setPower(frontLeftPower);
        this.frontRightMotor.setPower(frontRightPower);
        this.backRightMotor.setPower(backRightPower);
        telemetry.addData("backLeft: ", backLeftPower);
        telemetry.addData("backRight: ", backRightPower);
        telemetry.addData("frontLeft: ", frontLeftPower);
        telemetry.addData("frontRight", frontRightPower);
        telemetry.update();
    }

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }
}
