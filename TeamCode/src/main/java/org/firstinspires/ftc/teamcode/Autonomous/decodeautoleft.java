package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


@Autonomous
public class decodeautoleft extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    DcMotor rightRear;

    @Override
    public void runOpMode() {
        // Initialize your motors. Make sure the names match your configuration.
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "BR");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-48.6, -48.6, Math.toRadians(53));
        drive.setPoseEstimate(startPose);

        // Set the direction for motors if necessary
        // leftFront.setDirection(DcMotor.Direction.FORWARD);
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // ...

        waitForStart();

        if (opModeIsActive()) {
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(233)))
                    .build();
            drive.followTrajectory(traj1);







        }
    }


}