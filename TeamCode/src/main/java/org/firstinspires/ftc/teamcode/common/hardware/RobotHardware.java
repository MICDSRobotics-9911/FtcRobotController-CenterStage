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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
public class RobotHardware {
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx extensionMotor;
    public DcMotorEx intakeMotor;
    public Servo airplaneLaunch;
    public Servo airplaneHold;
    public static double holdMin = 0.25;
    public static double holdMax = 0.4;
    public static double launchMin = 0;
    public static double launchMax = 0.8;

    public ServoEx clawServo;
    public WebcamName camera;
    public WEncoder extensionEncoder;
    public static double serverMin = 0.1;
    public static double serverMax = 0.4;
    public WActuatorGroup extensionActuator;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private static RobotHardware instance = null;
    public boolean enabled;
    public Servo server;
    public List<LynxModule> modules;
    private ArrayList<WSubsystem> subsystems;
    public MecanumDrivetrain drivetrain;
    public IMU imu;
    private double imuAngle = 0;
    public DcMotorEx hangHolder;
    public DcMotorEx spoolHangMotor;


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


        if (!Globals.IS_AUTO) {
            this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
            //backLeftMotor.resetDeviceConfigurationForOpMode();
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            this.backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");
            //backRightMotor.resetDeviceConfigurationForOpMode();
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
            //frontLeftMotor.resetDeviceConfigurationForOpMode();
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
            //frontRightMotor.resetDeviceConfigurationForOpMode();
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            drivetrain = new MecanumDrivetrain(this.telemetry);
        }
        // DRIVETRAIN

        if (Globals.IS_USING_IMU) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
            imu = hardwareMap.get(IMU.class, "imu 1");
            imu.initialize(parameters);
            imu.resetYaw();
        }

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
        airplaneLaunch = hardwareMap.get(Servo.class, "airplane_launch");
        airplaneLaunch.setDirection(Servo.Direction.REVERSE);
        airplaneLaunch.scaleRange(launchMin, launchMax);

        airplaneHold = hardwareMap.get(Servo.class, "airplane_hold");
        airplaneHold.setDirection(Servo.Direction.REVERSE);
        airplaneHold.scaleRange(holdMin, holdMax);


        // Hang

        hangHolder = hardwareMap.get(DcMotorEx.class, "hang_holder");
        hangHolder.setDirection(DcMotorEx.Direction.REVERSE);
        hangHolder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangHolder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spoolHangMotor = hardwareMap.get(DcMotorEx.class, "spool_hang_motor");
        spoolHangMotor.setDirection(DcMotorEx.Direction.FORWARD);
        modules = hardwareMap.getAll(LynxModule.class);

        server = hardwareMap.get(Servo.class, "server");
        server.setDirection(Servo.Direction.REVERSE);
        server.scaleRange(serverMin, serverMax);

        this.subsystems = new ArrayList<>();
        if (Globals.IS_AUTO) {
            // some sort of localizer init
        } else {
            modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            // drone = new DroneSubsystem();
            // hang = new HangSubsystem();
        }
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
        if (Globals.IS_USING_IMU) {
            imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
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

    public double getVoltage() {
        return voltage;
    }

    public double getAngle() {
        return imuAngle;
    }

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }
}
