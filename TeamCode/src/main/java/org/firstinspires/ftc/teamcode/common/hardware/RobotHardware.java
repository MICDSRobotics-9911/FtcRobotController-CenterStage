package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RobotHardware {
    private double p = 0.0, i = 0.0, d = 0.0;
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

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }
}
