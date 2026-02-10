package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Normal robot constraints
                .setConstraints(60, 60,
                        Math.toRadians(180), Math.toRadians(180),
                        15)
                .followTrajectorySequence(drive -> {

                    // -------- SLOW PURGE CONSTRAINTS --------
                    TrajectoryVelocityConstraint slowVel =
                            new TranslationalVelocityConstraint(20);

                    TrajectoryAccelerationConstraint slowAccel =
                            new ProfileAccelerationConstraint(20);

                    return drive.trajectorySequenceBuilder(
                                    new Pose2d(-48.6, 48.6, Math.toRadians(127)))

                            // ===== SHOOT 1 =====
                            .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                            .waitSeconds(1.5)

                            // ===== PURGE 1 (SLOW) =====
                            .lineToSplineHeading(new Pose2d(-12, 18, Math.toRadians(90)))
                            .waitSeconds(0.2)
                            .setConstraints(slowVel, slowAccel)
                            .lineToSplineHeading(new Pose2d(-12, 50, Math.toRadians(90)))
                            .resetConstraints()
                            .waitSeconds(0.2)



                               //deschidem poarta
                            .lineToSplineHeading(new Pose2d(0, 20, Math.toRadians(180)))
                            .waitSeconds(0.2)

                            .lineToSplineHeading(new Pose2d(0, 50, Math.toRadians(180)))
                            .waitSeconds(0.2)

                            // ===== SHOOT 2 =====
                            .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                            .waitSeconds(1.5)

                            // ===== PURGE 2 (SLOW) =====
                            .lineToSplineHeading(new Pose2d(12, 18, Math.toRadians(90)))
                            .waitSeconds(0.2)
                            .setConstraints(slowVel, slowAccel)
                            .lineToSplineHeading(new Pose2d(12, 50, Math.toRadians(90)))
                            .resetConstraints()
                            .waitSeconds(0.2)

                            // ===== SHOOT 3 =====
                            .lineToSplineHeading(new Pose2d(12, 18, Math.toRadians(90)))
                            .waitSeconds(0.2)
                            .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                            .waitSeconds(1.5)

                            // ===== PURGE 3 (SLOW) =====
                            .lineToSplineHeading(new Pose2d(36, 18, Math.toRadians(90)))
                            .waitSeconds(0.2)
                            .setConstraints(slowVel, slowAccel)
                            .lineToSplineHeading(new Pose2d(36, 50, Math.toRadians(90)))
                            .resetConstraints()
                            .waitSeconds(0.2)

                            // ===== FINAL MOVE =====
                            .lineToSplineHeading(new Pose2d(0, 30, Math.toRadians(180)))

                            .build();

                });

        Image img = null;
        try {
            img = ImageIO.read(new File("C:/Users/Modus/Desktop/f/field.png"));
        } catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
