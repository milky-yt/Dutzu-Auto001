package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Basket Full", group = "Blue")
public class DutzuBlueBasket extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-48.6, -48.6, Math.toRadians(53));
        drive.setPoseEstimate(startPose);

        // -------- SLOW PURGE CONSTRAINTS --------
        TrajectoryVelocityConstraint slowVel =
                new TranslationalVelocityConstraint(20);

        TrajectoryAccelerationConstraint slowAccel =
                new ProfileAccelerationConstraint(20);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startPose)

                        // ===== SHOOT 1 =====
                        .lineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(233)))
                        .waitSeconds(1.5)

                        // ===== PURGE 1 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(-12, -18, Math.toRadians(270)))
                        .waitSeconds(0.2)
                        .setConstraints(slowVel, slowAccel)
                        .lineToSplineHeading(new Pose2d(-12, -50, Math.toRadians(270)))
                        .resetConstraints()
                        .waitSeconds(0.2)

                        // ===== SHOOT 2 =====
                        .lineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(233)))
                        .waitSeconds(1.5)

                        // ===== PURGE 2 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(12, -18, Math.toRadians(270)))
                        .waitSeconds(0.2)
                        .setConstraints(slowVel, slowAccel)
                        .lineToSplineHeading(new Pose2d(12, -50, Math.toRadians(270)))
                        .resetConstraints()
                        .waitSeconds(0.2)

                        // ===== SHOOT 3 =====
                        .lineToSplineHeading(new Pose2d(12, -18, Math.toRadians(270)))
                        .waitSeconds(0.2)
                        .lineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(233)))
                        .waitSeconds(1.5)

                        // ===== PURGE 3 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(36, -18, Math.toRadians(270)))
                        .waitSeconds(0.2)
                        .setConstraints(slowVel, slowAccel)
                        .lineToSplineHeading(new Pose2d(36, -50, Math.toRadians(270)))
                        .resetConstraints()
                        .waitSeconds(0.2)

                        // ===== FINAL MOVE =====
                        .lineToSplineHeading(new Pose2d(0, -30, Math.toRadians(180)))

                        .build()
        );
    }
}
