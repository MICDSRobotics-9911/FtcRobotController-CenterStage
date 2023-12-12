package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;

@Autonomous(name="RedAuto", group="Auto")
public class RedAuto extends LinearOpMode {
    public int element_zone = 1;
    private TeamElementSubsystem teamElementDetection = null;
    boolean togglePreview = true;

    public void hardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();
        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareStart();
        Side curAlliance = Side.RED;
        while (opModeIsActive() && !isStopRequested()) {
            teamElementDetection.setAlliance(curAlliance);
            int zone = teamElementDetection.elementDetection(telemetry);
            switch (zone) {
                case 1:
                    // Put trajectory to case 1 here
                    telemetry.addData("Case: ", 1);
                    break;
                case 2:
                    // Put trajectory to case 2 here
                    telemetry.addData("Case: ", 2);
                    break;
                default:
                    // Put trajectory to case 3 here
                    telemetry.addData("Case: ", 3);
                    break;
            }
            telemetry.addData("Current Alliance Selected : ", "red");
            telemetry.update();
        }
        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();
    }
}
