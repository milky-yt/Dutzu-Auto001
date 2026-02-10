package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Red Basket", group = "Red")
public class DutzuRedBasket_curve extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Flipped start position with mirrored heading
        Pose2d startPose = new Pose2d(-48.6, 48.6, Math.toRadians(127));
        drive.setPoseEstimate(startPose);

        // -------- FAST CONSTRAINTS --------
        TrajectoryVelocityConstraint fastVel =
                new TranslationalVelocityConstraint(60);

        TrajectoryAccelerationConstraint fastAccel =
                new ProfileAccelerationConstraint(57);



        // -------- SLOW CONSTRAINTS --------
        TrajectoryVelocityConstraint slowVel =
                new TranslationalVelocityConstraint(30);

        TrajectoryAccelerationConstraint slowAccel =
                new ProfileAccelerationConstraint(27);


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startPose)

                        // ===== SHOOT 1 =====
                        .setConstraints(fastVel, fastAccel) //repede

                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                        .waitSeconds(0.75)

                        // ===== PURGE 1 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(-12, 18, Math.toRadians(90)))


                        .resetConstraints()
                        .setConstraints(slowVel, slowAccel) //slow



                        .lineToSplineHeading(new Pose2d(-12, 47, Math.toRadians(90)))



                    //deschidem poarta
                        .lineToSplineHeading(new Pose2d(7, 40, Math.toRadians(180)))

                        .lineToSplineHeading(new Pose2d(7, 55, Math.toRadians(180)))



                        // ===== SHOOT 2 =====
                        .resetConstraints()
                        .setConstraints(fastVel, fastAccel) //repede

                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                        .waitSeconds(0.75)

                        // ===== PURGE 2 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(12, 15, Math.toRadians(90)))


                        .resetConstraints()
                        .setConstraints(slowVel, slowAccel)


                        .lineToSplineHeading(new Pose2d(22, 55, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(22, 40, Math.toRadians(90)))



                        // ===== SHOOT 3 =====
                        .resetConstraints()
                        .setConstraints(fastVel, fastAccel) //repede


                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                        .waitSeconds(0.75)

                        // ===== PURGE 3 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(36, 10, Math.toRadians(90)))



                        .resetConstraints()
                        .setConstraints(slowVel, slowAccel)


                        .lineToSplineHeading(new Pose2d(36, 55, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(90)))


                        //SHOOT FINAL
                        .resetConstraints()
                        .setConstraints(fastVel, fastAccel)
                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                        .waitSeconds(0.75)




                        .build()
        );
    }
}
