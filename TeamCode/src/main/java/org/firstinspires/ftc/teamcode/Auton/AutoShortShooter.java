package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Autonomous Mode: Short Red Auto Shooter
 * Class: AutoShortShooter
 *
 * This autonomous routine is designed for short-distance shooting from the red alliance side.
 * Sequence:
 *  1. Drive forward 10 inches
 *  2. Strafe right 10 inches
 *  3. Turn ~45 degrees clockwise
 *  4. Spin up shooter and feed rings
 *  5. Stop everything
 */

@Autonomous(name = "Short Red Auto Shooter", group = "Autonomous")
public class AutoShortShooter extends NextFTCOpMode {

    // Shooter hardware
    private ShootingSystem shootingSystem;
    private ShootingDirectionServo shootingDirectionServo;
    private BallLoadingServo ballLoadingServo;
    private Intake intakeSystem;

    // Drivetrain hardware (mecanum or tank-style with 4 motors)
    // IMPORTANT: make sure these names match the names in the robot config
    private final MotorEx frontLeft = new MotorEx(LEFT_FRONT_MOTOR_NAME).reversed();
    private final MotorEx frontRight = new MotorEx(RIGHT_FRONT_MOTOR_NAME);
    private final MotorEx backLeft = new MotorEx(LEFT_REAR_MOTOR_NAME).reversed();
    private final MotorEx backRight = new MotorEx(RIGHT_REAR_MOTOR_NAME);

    // --- TUNING CONSTANTS (adjust on your robot) ---
    private static final double TICKS_PER_REV = 537.6;          // goBILDA 312 RPM as example
    private static final double WHEEL_DIAMETER_INCHES = 3.78;    // change to your wheel size
    private static final double ROBOT_TURN_CPR = 1000.0;         // pseudo counts-per-radian for turn, tune on field

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES));

    @Override
    public void onInit() {
        // Shooter mech
        // We connect our subsystems to the robot and give them access to telemetry (data shown on driver station)
        shootingSystem = ShootingSystem.getInstance(telemetry);
        intakeSystem = Intake.getInstance(telemetry);
        shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        ballLoadingServo = BallLoadingServo.getInstance(telemetry);

        // Reset encoders and prepare for RUN_TO_POSITION style control
        initDriveMotor(frontLeft);
        initDriveMotor(frontRight);
        initDriveMotor(backLeft);
        initDriveMotor(backRight);

        addComponents(
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(intakeSystem),
                new SubsystemComponent(shootingDirectionServo),
                new SubsystemComponent(ballLoadingServo),
                //new SubsystemComponent(light),
                BulkReadComponent.INSTANCE,   // reads all sensors at once for faster updates
                BindingsComponent.INSTANCE    // helps connect buttons on the gamepads to commands
        );

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    /**
     * Helper to zero encoders and set mode.
     */
    private void initDriveMotor(MotorEx m) {
        m.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Anuj: it was RUN-TO-POSITION mode and commented
        m.setPower(0);
    }

    /**
     * Drive straight forward (+inches) / backward (-inches).
     */
    private void driveForwardInches(double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);

        // Each wheel same target for straight drive
        setTarget(frontLeft, ticks, power);
        setTarget(frontRight, ticks, power);
        setTarget(backLeft, ticks, power);
        setTarget(backRight, ticks, power);

        waitForDriveToFinish();
    }

    /**
     * Strafe right (+inches) / left (-inches) for mecanum.
     * NOTE: This assumes standard X-config mecanum.
     */
    private void strafeRightInches(double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);

        // Mecanum strafe: wheels get different polarities
        setTarget(frontLeft,  ticks,  power);
        setTarget(frontRight, -ticks, power);
        setTarget(backLeft,  -ticks, power);
        setTarget(backRight,  ticks,  power);

        waitForDriveToFinish();
    }

    /**
     * Turn clockwise by `radians` (negative for CCW).
     * We use ROBOT_TURN_CPR as an approximate ticks per radian.
     */
    private void turnRadians(double radians, double power) {
        int ticks = (int) Math.round(radians * ROBOT_TURN_CPR);

        // clockwise turn: left side forward, right side backward
        setTarget(frontLeft,  ticks,  power);
        setTarget(backLeft,   ticks,  power);
        setTarget(frontRight, -ticks, power);
        setTarget(backRight,  -ticks, power);

        waitForDriveToFinish();
    }

    /**
     * Low-level helper to assign target and power in RUN_TO_POSITION mode.
     */
    private void setTarget(MotorEx m, int targetTicks, double power) {
        // Current position + relative target
        //int newTarget = m.getCurrentPosition() + targetTicks;
        int newTarget = (int)m.getCurrentPosition() + targetTicks; //Anuj: new target should be double
        m.getMotor().setTargetPosition(newTarget);
        m.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(Math.abs(power));
    }

    /**
     * Block until all four drive motors are done moving (or opmode stops).
     */
    private void waitForDriveToFinish() {
        // Loop until all motors report !isBusy OR the opmode is stopped
        while (opModeIsActive() &&
                (frontLeft.getMotor().isBusy()
                        || frontRight.getMotor().isBusy()
                        || backLeft.getMotor().isBusy()
                        || backRight.getMotor().isBusy())) {

            telemetry.addData("fl/fr/bl/br", "%f / %f / %f / %f",
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
            sleep(10); //ANuj: Added this to give room for processing anything waiting for this thread to complete
        }

        // stop drive power after motion completes
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void onStartButtonPressed() {
        // === 1. Drive forward 10 inches ===
        driveForwardInches(30.0, 0.4);
        telemetry.addLine("Step 1: Drove forward 10 inches");
        telemetry.update();

        // === 2. Strafe right 10 inches ===
        strafeRightInches(20.0, 0.4);
        telemetry.addLine("Step 2: Strafed right 10 inches");
        telemetry.update();

        // === 3. Turn ~45 degrees clockwise ===
        // ~45 degrees = PI/4 radians ≈ 0.785
        turnRadians(0.785, 0.4);
        telemetry.addLine("Step 3: Turned ~45 deg CW");
        telemetry.update();

        // === 4. Shoot sequence ===
        // Spin up shooter
        shootingSystem.setPower(0.25);
        telemetry.addLine("Shooter motor started");
        telemetry.update();

        // allow flywheel to reach speed (tune this)
        sleep(5000);

        // Feed rings
        ballLoadingServo.runForward();
        sleep(10000); // feed for 10s
        ballLoadingServo.stop();
        telemetry.addLine("Rings fed");
        telemetry.update();

        // Stop shooter
        shootingSystem.setPower(0.0);
        telemetry.addLine("Shooter stopped");
        telemetry.update();

        // Final safety stop
        stopOpMode();
        sleep(20000); // feed for 20s
    }

    public void stopOpMode() {
        shootingSystem.setPower(0.0);
        ballLoadingServo.stop();

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addLine("All motors stopped");
        telemetry.update();
    }
}
