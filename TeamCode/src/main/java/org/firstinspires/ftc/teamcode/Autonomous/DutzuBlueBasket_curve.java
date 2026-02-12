package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Basket Curve ", group = "Blue")
public class DutzuBlueBasket_curve extends LinearOpMode {
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-48.6, 48.6, Math.toRadians(127));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        // ================= SHOOT 1 =================
        Trajectory t1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                .build();

        drive.followTrajectory(t1);
        sleep(750);

        // ================= PURGE 1 =================
        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .lineToSplineHeading(new Pose2d(-12, 18, Math.toRadians(90)))
                .build();

        drive.followTrajectory(t2);

        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .lineTo(new Vector2d(-12, 47)) // smoother straight line
                .build();

        drive.followTrajectory(t3);

        // Gate open
        Trajectory t4 = drive.trajectoryBuilder(t3.end())
                .lineToSplineHeading(new Pose2d(7, 40, Math.toRadians(180)))
                .build();

        drive.followTrajectory(t4);

        Trajectory t5 = drive.trajectoryBuilder(t4.end())
                .lineTo(new Vector2d(7, 55))
                .build();

        drive.followTrajectory(t5);

        // ================= SHOOT 2 =================
        Trajectory t6 = drive.trajectoryBuilder(t5.end())
                .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                .build();

        drive.followTrajectory(t6);
        sleep(750);

        // ================= PURGE 2 =================
        Trajectory t7 = drive.trajectoryBuilder(t6.end())
                .lineToSplineHeading(new Pose2d(12, 15, Math.toRadians(90)))
                .build();

        drive.followTrajectory(t7);

        Trajectory t8 = drive.trajectoryBuilder(t7.end())
                .lineTo(new Vector2d(22, 55))
                .build();

        drive.followTrajectory(t8);

        Trajectory t9 = drive.trajectoryBuilder(t8.end())
                .lineTo(new Vector2d(22, 40))
                .build();

        drive.followTrajectory(t9);

        // ================= SHOOT 3 =================
        Trajectory t10 = drive.trajectoryBuilder(t9.end())
                .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                .build();

        drive.followTrajectory(t10);
        sleep(750);

        // ================= PURGE 3 =================
        Trajectory t11 = drive.trajectoryBuilder(t10.end())
                .lineToSplineHeading(new Pose2d(36, 10, Math.toRadians(90)))
                .build();

        drive.followTrajectory(t11);

        Trajectory t12 = drive.trajectoryBuilder(t11.end())
                .lineTo(new Vector2d(36, 55))
                .build();

        drive.followTrajectory(t12);

        Trajectory t13 = drive.trajectoryBuilder(t12.end())
                .lineTo(new Vector2d(36, 40))
                .build();

        drive.followTrajectory(t13);

        // ================= FINAL SHOOT =================
        Trajectory t14 = drive.trajectoryBuilder(t13.end())
                .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))
                .build();

        drive.followTrajectory(t14);
        sleep(750);
    }
}