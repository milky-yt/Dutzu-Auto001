package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@TeleOp(name="Teleop Decode", group = "codulete")
public class decode_code extends LinearOpMode {

    private SampleMecanumDrive drive;
    private DcMotorEx OUTTAKE, INTAKE;
    private Servo s1, s2, s3, s4;

    private static double ANGLE_CLOSE_ARCH = 0.9; //UNGH LUAT DE RAMPA DE OUTTAKE PT BOLTA, DIN ZONA DE LANGA COSURI
    private static double ANGLE_CLOSE_STRAIGHT = 0.6; //UNGH LUAT DE RAMPA DE OUTTAKE PT DREPT, DIN ZONA DE LANGA COSURI
    private static double ANGLE_FAR_ARCH = 0.9;   //UNGH LUAT DE RAMPA DE OUTTAKE PT BOLTA, DIN ZONA DE LANGA PARKING
    private static double ANGLE_FAR_STRAIGHT = 0.6;   //UNGH LUAT DE RAMPA DE OUTTAKE PT DREPT, DIN ZONA DE LANGA PARKING
    private static double LAUNCHER_CLOSE_ARCH = 1;
    private static double LAUNCHER_CLOSE_STRAIGHT = 1;
    private static double LAUNCHER_FAR_ARCH = 1;
    private static double LAUNCHER_FAR_STRAIGHT = 1;
    private static double LAUNCHER_POWER = 1; //initializat pt launcher close arch

    private static final double INTAKE_POWER = 0.6;
    private static final double SERVO_FORWARD = 1.0;


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
            debug_values(); //debug
            drive.update();
        }
    }

    private void initializeHardware() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        OUTTAKE = hardwareMap.get(DcMotorEx.class, "OUTTAKE");
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");

        s1 = hardwareMap.get(Servo.class, "s1"); // primu rand de servo
        s2 = hardwareMap.get(Servo.class, "s2"); //randu doi de servo


        s3 = hardwareMap.get(Servo.class, "s3"); // servo unghi outtake 1
        s4 = hardwareMap.get(Servo.class, "s4"); //servo unghi outtake 2
        s3.setPosition(ANGLE_CLOSE_ARCH);
        s4.setPosition(ANGLE_CLOSE_ARCH);
    }

    private void setupMotors() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void handleDrive() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (Math.abs(y) < 0.1) y = 0;
        if (Math.abs(x) < 0.1) x = 0;
        if (Math.abs(rx) < 0.1) rx = 0;

        Vector2d input = new Vector2d(y, x);

        drive.setWeightedDrivePower(
                new Pose2d(input.getX(),
                input.getY(),
                rx )
        );
    }

    private void handleIntake() {
        if (gamepad1.right_bumper) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                INTAKE.setPower(INTAKE_POWER);
            } else {
                INTAKE.setPower(0);
            }
            while (gamepad1.right_bumper) {
                sleep(10);
            }
        }
    }
    private void handleAngle(){
        if (gamepad2.dpad_up)        {
            s3.setPosition(ANGLE_CLOSE_ARCH); //valoare de dat cu bolta (DE APROAPE)
            s4.setPosition(ANGLE_CLOSE_ARCH);

            LAUNCHER_POWER = LAUNCHER_CLOSE_ARCH;
        }
        if (gamepad2.dpad_right)     {
            s3.setPosition(ANGLE_CLOSE_STRAIGHT); // valoare de dat drept (DE APROAPE)
            s4.setPosition(ANGLE_CLOSE_STRAIGHT);

            LAUNCHER_POWER = LAUNCHER_CLOSE_STRAIGHT;
        }
        if (gamepad2.dpad_left)     {
            s3.setPosition(ANGLE_FAR_ARCH); // valoare de dat cu bolta (DE DEPARTE)
            s4.setPosition(ANGLE_FAR_ARCH);

            LAUNCHER_POWER = LAUNCHER_FAR_ARCH;
        }
        if (gamepad2.dpad_down)        {
            s3.setPosition(ANGLE_FAR_STRAIGHT); //valoare de dat drept (DE DEPARTE)
            s4.setPosition(ANGLE_FAR_STRAIGHT);

            LAUNCHER_POWER = LAUNCHER_FAR_STRAIGHT;
        }
    }

    private void handleLaunching() {
        if (gamepad2.right_bumper) {
            OUTTAKE.setPower(LAUNCHER_POWER);
            isLaunching = true;
        }


        if (isLaunching) { //inseamna doar ca nu e toggle si ii dai timp de wind down
            //TODO: eventual o miscare de revert ca daca dai click din greseala sa nu lanseze / ca sa anuleze miscarea
            OUTTAKE.setPower(LAUNCHER_POWER);

            sleep(100);

            OUTTAKE.setPower(0);
            isLaunching=false;
        }
    }

    private void handleServoControl() {
        if (gamepad1.x) {
            s1.setPosition(SERVO_FORWARD);
            s2.setPosition(SERVO_FORWARD);

        }
    }




//DEBUG PT A GASI VALORI LA POWER SI LA ANGLE
    private void debug_values(){
        if (gamepad1.b){
            ANGLE_CLOSE_ARCH = ANGLE_CLOSE_ARCH + 0.01;
            ANGLE_FAR_ARCH = ANGLE_FAR_ARCH + 0.01;
            ANGLE_CLOSE_STRAIGHT = ANGLE_CLOSE_STRAIGHT +0.01;
            ANGLE_FAR_STRAIGHT = ANGLE_FAR_STRAIGHT + 0.01;
        }
        if (gamepad1.x){
            ANGLE_CLOSE_ARCH = ANGLE_CLOSE_ARCH - 0.01;
            ANGLE_FAR_ARCH = ANGLE_FAR_ARCH - 0.01;
            ANGLE_CLOSE_STRAIGHT = ANGLE_CLOSE_STRAIGHT - 0.01;
            ANGLE_FAR_STRAIGHT = ANGLE_FAR_STRAIGHT - 0.01;
        }


        if (gamepad1.y){
            LAUNCHER_CLOSE_ARCH = LAUNCHER_CLOSE_ARCH + 0.01;
            LAUNCHER_FAR_ARCH = LAUNCHER_FAR_ARCH + 0.01;
            LAUNCHER_CLOSE_STRAIGHT = LAUNCHER_CLOSE_STRAIGHT +0.01;
            LAUNCHER_FAR_STRAIGHT = LAUNCHER_FAR_STRAIGHT + 0.01;
        }
        if (gamepad1.a){
            LAUNCHER_CLOSE_ARCH = LAUNCHER_CLOSE_ARCH - 0.01;
            LAUNCHER_FAR_ARCH = LAUNCHER_FAR_ARCH - 0.01;
            LAUNCHER_CLOSE_STRAIGHT = LAUNCHER_CLOSE_STRAIGHT - 0.01;
            LAUNCHER_FAR_STRAIGHT = LAUNCHER_FAR_STRAIGHT - 0.01;
        }


    }



    private void updateTelemetry() {
        telemetry.addLine("=== FTC TeleOp ===");
        // debug
        telemetry.addLine("\n=== DEBUG Controls ===");
        telemetry.addLine("\n=== driver 1 cerc, creste putere spindex, patrat scade ===");
        telemetry.addData("Launcher FAR STRAIGHT", LAUNCHER_FAR_STRAIGHT);
        telemetry.addData("Launcher CLOSE STRAIGHT", LAUNCHER_CLOSE_STRAIGHT);
        telemetry.addData("Launcher FAR ARCH", LAUNCHER_FAR_ARCH);
        telemetry.addData("Launcher CLOSE ARCH", LAUNCHER_CLOSE_ARCH);
        //geniune
        telemetry.addData("Launch Status", isLaunching ? "LAUNCHING" : "Ready");
        telemetry.addData("Intake Status", intakeOn ? "ON" : "OFF");
        //debug
        telemetry.addLine("\n=== driver 1 triunghi, creste unghi outtake, x scade ===");
        telemetry.addData("ANGLE FAR STRAIGHT", ANGLE_FAR_STRAIGHT);
        telemetry.addData("ANGLE CLOSE STRAIGHT", ANGLE_CLOSE_STRAIGHT);
        telemetry.addData("ANGLE FAR ARCH", ANGLE_FAR_ARCH);
        telemetry.addData("ANGLE CLOSE ARCH", ANGLE_CLOSE_ARCH);



        telemetry.addLine("\n=== DRIVER 1 ===");
        telemetry.addLine("RIGHT BUMPER = Toggle Intake");
        telemetry.addLine("Sticks = Drive (with brake)");
        telemetry.addLine("\n=== DRIVER 2 ===");
        telemetry.addLine("RIGHT BUMPER = TOGGLE OUTTAKE, CLOSE STRAIGHT");
        telemetry.addLine("RIGHT TRIGGER = TOGGLE OUTTAKE, CLOSE ARCH");
        telemetry.addLine("LEFT BUMPER = TOGGLE OUTTAKE, FAR STRAIGHT");
        telemetry.addLine("LEFT TRIGGER = TOGGLE OUTTAKE, FAR ARCH");
        telemetry.addLine("PATRAT = Spin Servos");
        telemetry.addLine("dpad up = close arch");
        telemetry.addLine("dpad left = close straight");
        telemetry.addLine("dpad right = far arch");
        telemetry.addLine("dpad down = far straight");

        telemetry.update();
    }
}