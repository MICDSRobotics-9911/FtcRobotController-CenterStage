package org.firstinspires.ftc.teamcode.onbotjavafiles;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.arcrobotics.ftclib.controller.PIDController;
@TeleOp(name="LinearSlideTest")
public class LinearSlideTest extends LinearOpMode {
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    //private PIDController controller;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx linearSlideMotor;
    private final double ticks_in_degrees = 700 / 100.0;
    public static int target = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        /*backLeft = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");*/
        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linear_slide_motor");
        /*backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //controller = new PIDController(p, i, d);



        waitForStart();
        while (opModeIsActive()) {
            /*controller.setPID(p, i, d);
            int linearSlidePos = linearSlideMotor.getCurrentPosition();
            double pid = controller.calculate(linearSlidePos, target);
            double ff = 0;
            //double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double linearSlidePower = pid + ff;*/
            linearSlideMotor.setTargetPosition(target);

            if (gamepad1.x) {
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(-1);
            }

            if (linearSlideMotor.getCurrentPosition() > target) {
                linearSlideMotor.setPower(0);
            }
            /*double x, y, power, turn, theta, bl, br, fl, fr;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            power = Math.hypot(x, y);
            turn = gamepad1.right_stick_x;
            theta = Math.atan2(y, x);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            bl = power * sin/max + turn;
            br = power * cos/max - turn;
            fl = power * cos/max + turn;
            fr = power * sin/max - turn;

            if ((power + Math.abs(turn)) > 1) {
                fl /= power + turn;
                fr /= power + turn;
                bl /= power + turn;
                br /= power + turn;
            }
            backLeft.setPower(bl);
            backRight.setPower(br);
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            telemetry.addData("backLeft: ", bl);
            telemetry.addData("backRight: ", br);
            telemetry.addData("frontLeft: ", fl);
            telemetry.addData("frontRight", fr);*/
            telemetry.update();
        }
    }
}