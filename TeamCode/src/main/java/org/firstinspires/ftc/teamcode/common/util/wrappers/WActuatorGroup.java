package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

import java.util.HashMap;
import java.util.Map;

public class WActuatorGroup {
    private final Map<String, HardwareDevice> devices = new HashMap<>();
    public ElapsedTime timer;
    private RobotHardware robot = RobotHardware.getInstance();
    private PIDController controller;
    private double power = 0.0;
    private double position = 0.0;
    private double targetPosition = 0.0;
    private double targetPositionOffset = 0.0;
    private double currentFeedForward = 0.0;
    private double offset = 0.0;
    private double tolerance = 0.0;
    public WActuatorGroup(HardwareDevice... devices) {
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();

    }

    public void read() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof WEncoder) {
                this.position = (int) (((WEncoder) device).getPosition() + offset);
                return;
            }
        }
        this.position = 0.0;
    }

    public void periodic() {
        if (timer == null) {
            timer = new ElapsedTime();
        }
        if (controller != null) {
            this.power = controller.calculate(position, targetPosition + targetPositionOffset);
            this.power += currentFeedForward * Math.signum((targetPosition + targetPositionOffset) - position);
            this.power = MathUtils.clamp(power, -1, 1);
        }
    }

    public void write() {
        int i = 0;
        for (HardwareDevice device : devices.values()) {
            if (device instanceof DcMotor) {
                double correction = 1.0;
                //if (voltage != null) correction = 12.0 / voltage.getAsDouble();
            } else if (device instanceof Servo) {
                ((Servo) device).setPosition(targetPosition);
            }
        }
    }

    public WActuatorGroup setPIDController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    public WActuatorGroup setErrorTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public double getPosition() {
        return position;
    }
}
