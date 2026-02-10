package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Test Rotate Clockwise", group = "Test")
public class rotate extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Start pose (anywhere you want)
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        // Rotate 90 degrees clockwise
        drive.turn(Math.toRadians(-90)); // negative for clockwise in Roadrunner

        // Optional: hold for a few seconds to see the result
        sleep(2000);
    }
}
