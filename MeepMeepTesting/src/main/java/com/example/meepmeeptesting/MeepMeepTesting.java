package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.DriveConstants.MAX_ACCEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_ANG_ACCEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_ANG_VEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_VEL;
import static com.example.meepmeeptesting.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;




public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(100));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .turn(Math.toRadians(-10))
                                .forward(27)
                                .turn(Math.toRadians(90))
                                .forward(8)
                                .back(20)
                                .turn(Math.toRadians(180))
                                .forward(26)
                                .strafeLeft(3)
                                .addDisplacementMarker(() -> {
                                    //drive.resetHeadingPID();
                                })
                                .strafeLeft(2)
                                .addDisplacementMarker(() -> {
                                    // Drop Yellow Pixel on backdrop
                                    //robot.server.setPosition(1);
                                })
                                .strafeRight(30)
                                .addDisplacementMarker(() -> {
                                    // Reset Yellow Pixel on backdrop
                                    //robot.server.setPosition(0);
                                })
                                .forward(5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}