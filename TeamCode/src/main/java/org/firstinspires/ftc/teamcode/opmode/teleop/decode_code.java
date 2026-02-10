package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@TeleOp(name="decode code", group = "codulete")
public class decode_code extends LinearOpMode {

    private SampleMecanumDrive drive;
    private DcMotorEx LL, LR, INTAKE;
    private Servo s1, s2, s3, s4, s5, s6;

    private static final double LAUNCHER_POWER = 0.75;
    private static final double INTAKE_POWER = 1.0;
    private static final double SERVO_FORWARD = 1.0;
    private static final double SERVO_REVERSE = 0.0;
    private static final double SERVO_STOP = 0.5;

    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean isLaunching = false;
    private boolean intakeOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        setupMotors();

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleIntake();
            handleLaunching();
            handleAngle();
            handleServoControl();
            updateTelemetry();
            drive.update();
        }
    }

    private void initializeHardware() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        LL = hardwareMap.get(DcMotorEx.class, "LL");
        LR = hardwareMap.get(DcMotorEx.class, "LR");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");
    }

    private void setupMotors() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void handleDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (Math.abs(y) < 0.1) y = 0;
        if (Math.abs(x) < 0.1) x = 0;
        if (Math.abs(rx) < 0.1) rx = 0;

        Vector2d input = new Vector2d(y, x);

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        rx
                )
        );
    }

    private void handleIntake() {
        if (gamepad1.y) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                INTAKE.setPower(INTAKE_POWER);
            } else {
                INTAKE.setPower(0);
            }
            while (gamepad1.y) {
                sleep(10);
            }
        }
    }
    private void handleAngle(){
        if (gamepad2.dpad_up)        {
            s1.setPosition(0.6);
            s4.setPosition(0.6);
        }
        if ((gamepad2.dpad_left) || (gamepad2.dpad_right))    {
            s1.setPosition(0.4);
            s4.setPosition(0.4);
        }
        if (gamepad2.dpad_down)        {
            s1.setPosition(0.2);
            s4.setPosition(0.2);
        }
    }

    private void handleLaunching() {
        if (gamepad1.right_bumper && !isLaunching) {
            isLaunching = true;
            launchTimer.reset();

            LL.setPower(LAUNCHER_POWER);
            LR.setPower(-LAUNCHER_POWER);
        }

        if (isLaunching && launchTimer.seconds() <= 2.0) {
            LL.setPower(LAUNCHER_POWER);
            LR.setPower(-LAUNCHER_POWER);

            if (launchTimer.seconds() >= 0.5 && launchTimer.seconds() < 2.0) {
                s2.setPosition(SERVO_FORWARD);
                s3.setPosition(SERVO_REVERSE);

                s5.setPosition(SERVO_REVERSE);
                s6.setPosition(SERVO_REVERSE);
            }
        }

        if(isLaunching && launchTimer.seconds() > 2.0) {
            isLaunching = false;
        }

        if (!gamepad1.right_bumper && !isLaunching) {
            LL.setPower(0);
            LR.setPower(0);
        }
    }

    private void handleServoControl() {
        if (gamepad1.x) {
            s2.setPosition(SERVO_FORWARD);
            s3.setPosition(SERVO_REVERSE);

            s5.setPosition(SERVO_REVERSE);
            s6.setPosition(SERVO_REVERSE);
        } else if (!isLaunching) {
            stopAllServos();
        }
    }

    private void stopAllServos() {
        s2.setPosition(SERVO_STOP);
        s3.setPosition(SERVO_STOP);

        s5.setPosition(SERVO_STOP);
        s6.setPosition(SERVO_STOP);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== FTC TeleOp ===");
        telemetry.addData("Launcher Power", LL.getPower());
        telemetry.addData("Launch Status", isLaunching ? "LAUNCHING" : "Ready");
        telemetry.addData("Intake Status", intakeOn ? "ON" : "OFF");

        telemetry.addLine("\n=== Controls ===");
        telemetry.addLine("Right Bumper = Launch Sequence");
        telemetry.addLine("Y = Toggle Intake");
        telemetry.addLine("X = Spin Servos");
        telemetry.addLine("Sticks = Drive (with brake)");

        telemetry.update();
    }
}