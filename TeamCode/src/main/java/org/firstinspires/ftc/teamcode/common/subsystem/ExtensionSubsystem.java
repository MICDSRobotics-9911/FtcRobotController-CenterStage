package org.firstinspires.ftc.teamcode.common.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
@Config
public class ExtensionSubsystem extends WSubsystem {
    private final RobotHardware robot;
    private int backdropHeight = 0;
    private boolean scoring = false;
    private boolean updated = false;

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        double liftTicks = robot.extensionEncoder.getPosition();
        robot.extensionActuator.periodic();
    }

    @Override
    public void read() {
        robot.extensionActuator.read();
    }

    @Override
    public void write() {
        robot.extensionActuator.write();
    }

    @Override
    public void reset() {

    }

    public void setBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(amount, 0, 11);
        updated = false;
    }
    public int getBackdropHeight() {
        return backdropHeight;
    }

    public void setScoring(boolean scoring) {
        this.scoring = scoring;
    }

    public boolean getScoring() {
        return this.scoring;
    }

    public void setUpdated(boolean updated) {
        this.updated = updated;
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(getBackdropHeight() + amount, 0, 11);
        updated = false;
    }


}
