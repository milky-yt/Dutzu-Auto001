package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Preload ", group = "Blue")
public class DutzuBluePreload extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Flipped start position with mirrored heading
        Pose2d startPose = new Pose2d(-48.6, -48.6, Math.toRadians(307));
        drive.setPoseEstimate(startPose);



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startPose)

                        // ===== SHOOT 1 =====
                        .lineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(233)))
                        .waitSeconds(1.5)

                        .build()
        );
    }
}
