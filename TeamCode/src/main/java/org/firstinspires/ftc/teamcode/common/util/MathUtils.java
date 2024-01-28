package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;


public class MathUtils {
    private static final double camxoffset = 0;
    private static final double camyoffset = 0;
    private static AprilTagLibrary tags = AprilTagGameDatabase.getCurrentGameTagLibrary();
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static Pose2d getFCPosition(List<AprilTagDetection> detections, double botheading) {
        Pose2d ret;
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC


        double avgX = 0;
        double avgY = 0;
        for (AprilTagDetection detection : detections) {
            double x = (detection.ftcPose.x + 1.5)-camxoffset;
            double y = detection.ftcPose.y-camyoffset;

            // invert heading to correct properly
            botheading = -botheading;

            // rotate RC coordinates to be field-centric
            double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
            double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);

            // add FC coordinates to apriltag position
            // tags is just the CS apriltag library
            VectorF tagpose = tags.lookupTag(detection.id).fieldPosition;

            avgX +=  tagpose.get(0)+y2;
            avgY += tagpose.get(1)-x2;
        }
        return new Pose2d(avgX / detections.size(), avgY / detections.size(), -botheading);
        //return new Pose2d(tagpose.get(0)+y2,tagpose.get(1)-x2,-botheading);
    }

}
