package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Preload Back", group = "Blue")
public class DutzuBluePreloadBack extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Flipped start position with mirrored heading
        Pose2d startPose = new Pose2d(61.5, -12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startPose)

                        // ===== SHOOT 1 =====
                        .lineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(127)))
                        .waitSeconds(1.5)

                        .build()
        );
    }
}
