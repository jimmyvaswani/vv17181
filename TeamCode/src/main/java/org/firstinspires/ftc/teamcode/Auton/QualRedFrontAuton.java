package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

@Autonomous(name = "Qual Short Red Auto", group = "Autonomous")
public class QualRedFrontAuton extends LinearOpMode {

    // Drive + shooter hardware
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private Intake intakeSystem;
    private ShootingSystem shootingSystem;
    private BallLoadingServo ballPusherMotor;
    private ShootingDirectionServo shootingDirectionServo;


    private boolean bShooterRunning = false;

    // Encoder math
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 312RPM (NEVEREST-style)
    private static final double DRIVE_GEAR_REDUCTION = 1.0;   // no external gearing
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // wheel diameter
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double LAUNCHER_MOTOR_TICKS_PER_REV = 28.0; // typical REV motor encoder CPR

    // Timing constants (ms)
    private static final long INITIAL_SPIN_UP_TIME = 1300;
    private static final long GATE_DOWN_TIME = 500;
    private static final long GATE_UP_TIME = 800;
    private static final long MINIMAL_SLEEP_TIME = 1;

    // Servo stop gate positions
    private static final double STOPPER_CLOSED = 0.70;
    private static final double STOPPER_OPEN = 0.94;

    /**
     * Main entry point for the autonomous mode.
     * Initializes hardware, waits for start signal, and executes the autonomous sequence.
     */
    @Override
    public void runOpMode() {
        // === init() equivalent ===
        leftFront = hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        rightFront = hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        leftBack = hardwareMap.dcMotor.get(LEFT_REAR_MOTOR_NAME);
        rightBack = hardwareMap.dcMotor.get(RIGHT_REAR_MOTOR_NAME);

        intakeSystem = Intake.getInstance(telemetry);
        shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        ballPusherMotor = BallLoadingServo.getInstance(telemetry);
        shootingSystem = ShootingSystem.getInstance(telemetry);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();

        // Wait for Start button on Driver Hub
        waitForStart();
        telemetry.addData("Status", "Auton Code Started");
        telemetry.update();
//        sleep(5000);

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running Commands in Sequence");
            telemetry.update();
            intakeSystem.start();
            sleep(5000);

            shootingDirectionServo.setPosition(0.6);
            sleep(2000);
            // === AUTON STEPS GO HERE ===
            // Example sequence:
            // 1. Spin up shooter & fire
            // shootPowerCore(1.0, false, 3200);
            // 2. Drive forward 24 inches
            encoderDrive(0.4, 24, 24, 3.0);
            sleep(2000);

            shootingSystem.setPower(1.0);
            sleep(2000);
            // Add your real path here.
            ballPusherMotor.runForward();
            sleep(10000);
        }

        // When opmode ends, we'll drop out of runOpMode()
        stopAllMotors();
        telemetry.addData("Status", "Auton Code Stopped");
        telemetry.update();
    }

    /**
     * Starts the shooter motor and ball pusher with specified power and velocity.
     */
    public void startShooter(double launcherPower, double ballPusherVelocity) {
        shootingSystem.setPower(launcherPower);
        ballPusherMotor.runForward(ballPusherVelocity);
        bShooterRunning = true;
    }

    /**
     * Controls the shooting sequence by spinning up the launcher, opening and closing the gate servo,
     * and firing a specified number of balls.
     */
    public void shootPowerCore(double launcherPower, boolean unused, double ballPusherVelocity) {
        if (!bShooterRunning) {
            shootingSystem.setPower(launcherPower);
            ballPusherMotor.runForward(ballPusherVelocity);
            sleep(INITIAL_SPIN_UP_TIME);
            bShooterRunning = true;
        }

        // Fire 4 rings/balls
        for (int i = 0; i < 4 && opModeIsActive(); i++) {
            //stopperServo.setPosition(STOPPER_OPEN);
            sleep(GATE_DOWN_TIME);

            //stopperServo.setPosition(STOPPER_CLOSED);
            sleep(GATE_UP_TIME);
        }

        shootingSystem.setPower(0);
        ballPusherMotor.stop();
        bShooterRunning = false;
    }

    /**
     * Turns on the intake system by setting the intake and ball pusher motor velocities.
     */
    public void turnOnIntake(double intakeVelocity, double ballPusherVelocity) {
        intakeSystem.start();
        ballPusherMotor.runForward(ballPusherVelocity);
    }

    /**
     * Turns off the intake and ball pusher motors.
     */
    public void turnOffIntake() {
        intakeSystem.stop();
        ballPusherMotor.stop();
    }

    /**
     * Sets all drive motors to the provided RunMode (e.g., RUN_TO_POSITION or RUN_USING_ENCODER).
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    /**
     * Stops all drive motors by setting their power to zero.
     */
    private void stopAllMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        intakeSystem.stop();
    }

    /**
     * Drives the robot straight for specified left and right distances using encoders.
     * Stops when targets are reached or timeout expires.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        if (!opModeIsActive()) return;

        int leftFrontTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        rightBack.setTargetPosition(rightBackTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        leftFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Running to L:%7d R:%7d", leftFrontTarget, rightFrontTarget);
            telemetry.addData("Current", "Running at L:%7d R:%7d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.update();
            sleep(MINIMAL_SLEEP_TIME);
        }

        stopAllMotors();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Overloaded encoderDrive method allowing independent distances for each wheel.
     * Useful for strafing or turning maneuvers.
     */
    public void encoderDrive(double speed,
                             double lfInches, double rfInches,
                             double lbInches, double rbInches,
                             double timeoutS) {
        if (!opModeIsActive()) return;

        int lfTarget = leftFront.getCurrentPosition() + (int) (lfInches * COUNTS_PER_INCH);
        int rfTarget = rightFront.getCurrentPosition() + (int) (rfInches * COUNTS_PER_INCH);
        int lbTarget = leftBack.getCurrentPosition() + (int) (lbInches * COUNTS_PER_INCH);
        int rbTarget = rightBack.getCurrentPosition() + (int) (rbInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(lfTarget);
        rightFront.setTargetPosition(rfTarget);
        leftBack.setTargetPosition(lbTarget);
        rightBack.setTargetPosition(rbTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Strafing to targets");
            telemetry.addData("LF Pos", leftFront.getCurrentPosition());
            telemetry.addData("RF Pos", rightFront.getCurrentPosition());
            telemetry.addData("LB Pos", leftBack.getCurrentPosition());
            telemetry.addData("RB Pos", rightBack.getCurrentPosition());
            telemetry.update();
            sleep(MINIMAL_SLEEP_TIME);
        }

        stopAllMotors();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
