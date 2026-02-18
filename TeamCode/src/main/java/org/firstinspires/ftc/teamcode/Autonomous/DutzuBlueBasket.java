package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Basket", group = "Red")
public class DutzuBlueBasket extends LinearOpMode {
    DcMotorEx INTAKE;
    DcMotorEx OUTTAKE;
    Servo s1;
    Servo s2;
    Servo s3;
    Servo s4;
    Servo s5;
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");

        OUTTAKE = hardwareMap.get(DcMotorEx.class, "OUTTAKE");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        OUTTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OUTTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INTAKE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Flipped start position with mirrored heading
        Pose2d startPose = new Pose2d(-48.6, 48.6, Math.toRadians(127));
        drive.setPoseEstimate(startPose);

        // -------- FAST CONSTRAINTS --------
        TrajectoryVelocityConstraint fastVel =
                new TranslationalVelocityConstraint(40);

        TrajectoryAccelerationConstraint fastAccel =
                new ProfileAccelerationConstraint(40);



        // -------- SLOW CONSTRAINTS --------
        TrajectoryVelocityConstraint slowVel =
                new TranslationalVelocityConstraint(15);

        TrajectoryAccelerationConstraint slowAccel =
                new ProfileAccelerationConstraint(15);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startPose)

                        // ===== SHOOT 1 =====
                        .setConstraints(fastVel, fastAccel) //repede

                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))

                        //launcher ON|

                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> OUTTAKE.setPower(1))

                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s2.setPosition(1))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s3.setPosition(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s4.setPosition(1))

                        .waitSeconds(3)

                        .UNSTABLE_addTemporalMarkerOffset(0, () -> OUTTAKE.setPower(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s2.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s3.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s4.setPosition(0.5))



                        // ===== PURGE 1 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(-12, 25, Math.toRadians(90)))

                        .resetConstraints()
                        .setConstraints(slowVel, slowAccel) //slow


                        .addDisplacementMarker( () -> {
                            INTAKE.setPower(0.6);
                        })


                        .lineToSplineHeading(new Pose2d(-12, 57, Math.toRadians(90)))

                        .addDisplacementMarker(() -> {
                            INTAKE.setPower(0);
                        })


                        // ===== SHOOT 2 =====
                        .resetConstraints()
                        .setConstraints(fastVel, fastAccel) //repede

                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))

                        //launcher ON

                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s2.setPosition(1))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s3.setPosition(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s4.setPosition(1))

                        .waitSeconds(3)

                        .UNSTABLE_addTemporalMarkerOffset(0, () -> OUTTAKE.setPower(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s2.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s3.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s4.setPosition(0.5))



                        // ===== PURGE 2 (SLOW) =====
                        .lineToSplineHeading(new Pose2d(7, 20, Math.toRadians(90)))

                        .resetConstraints()
                        .setConstraints(slowVel, slowAccel)


                        .addDisplacementMarker( () -> {
                            INTAKE.setPower(0.6);
                        })

                        .lineToSplineHeading(new Pose2d(22, 65, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(22, 40, Math.toRadians(90)))


                        .addDisplacementMarker( () -> {
                            INTAKE.setPower(0);
                        })

                        // ===== SHOOT 3 =====
                        .resetConstraints()
                        .setConstraints(fastVel, fastAccel) //repede

                        .lineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(127)))


                        //launcher ON
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s2.setPosition(1))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s3.setPosition(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s4.setPosition(1))

                        .waitSeconds(3)

                        .UNSTABLE_addTemporalMarkerOffset(0, () -> OUTTAKE.setPower(0))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s2.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s3.setPosition(0.5))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s4.setPosition(0.5))



                        .build()
        );
    }
}