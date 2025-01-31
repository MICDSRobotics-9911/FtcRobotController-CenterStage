package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;

public class Globals {
    public static Side COLOR = Side.RED;
    // Match constants
    public static Side SIDE = Side.LEFT;
    public static boolean IS_AUTO = false;
    public static boolean IS_USING_IMU = false;
    public static boolean USING_DASHBOARD = false;

    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;

    public static void startScoring() {
        IS_SCORING = true;
        IS_INTAKING = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void retract() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }
}
