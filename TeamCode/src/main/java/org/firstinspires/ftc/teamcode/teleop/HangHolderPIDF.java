package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Disabled
@TeleOp(name="HangHolderPIDF", group="test")
public class HangHolderPIDF extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degrees =  288 / 180.0;
    private DcMotorEx hangHolder;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hangHolder = hardwareMap.get(DcMotorEx.class, "hang_holder");
        hangHolder.setDirection(DcMotor.Direction.REVERSE);
        hangHolder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangHolder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int hangPos = hangHolder.getCurrentPosition();
        double pid = controller.calculate(hangPos, target);
        double ff = Math.sin(Math.toRadians(hangPos / ticks_in_degrees + 30)) * f;
        double power = pid + ff;
        hangHolder.setPower(power);
        telemetry.addData("HangPos: ", hangPos);
        telemetry.addData("Target: ", target);
        telemetry.update();
    }
}
