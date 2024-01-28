package org.firstinspires.ftc.teamcode.common.centerstage;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class HelpfulAprilTagDetection {
    private static int minExposure = 0;
    private static int maxExposure = 0;
    private static int myGain;
    private static int minGain;
    private static int maxGain;
    public static void getCameraSetting(VisionPortal portal) {
        // Ensure Vision Portal has been setup.
        if (portal == null) {
            return;
        }
        // Get camera control values unless we are stopping.
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = portal.getCameraControl(GainControl.class);
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
    }

    public static boolean setManualExposure(VisionPortal visionPortal, int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open

        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        return (true);
    }
}
