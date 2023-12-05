package org.firstinspires.ftc.teamcode.onbotjavafiles;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="field-centric mecanum")
public class OtherFieldCentricTest extends LinearOpMode {

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    private PIDController controller;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotorEx linearSlideMotor;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linear_slide_motor");


        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();
        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int linearSlidePos = linearSlideMotor.getCurrentPosition();
            double pid = controller.calculate(linearSlidePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double linearSlidePower = pid + ff;
            linearSlideMotor.setTargetPosition(target);
            if (gamepad1.x) {
                linearSlideMotor.setPower(linearSlidePower);
            }
            fieldCentricPlus();
        }
    }

    public void fieldCentricPlus() {
        double x, y, power, rx, theta, botHeading, rotX, rotY;

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        if (gamepad1.options) {
            imu.resetYaw();
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        backLeft.setPower(backLeftPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}

