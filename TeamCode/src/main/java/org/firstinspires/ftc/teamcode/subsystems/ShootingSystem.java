    package org.firstinspires.ftc.teamcode.subsystems;

    import dev.nextftc.core.commands.Command;
    import dev.nextftc.core.commands.utility.InstantCommand;
    import dev.nextftc.core.subsystems.Subsystem;
    import dev.nextftc.hardware.impl.MotorEx;
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.teleop.RobotCentricTeleOp;

    /**
     * ShootingSystem Subsystem for the robot.
     *
     * This class controls the shooting mechanism using two motors with simple power control.
     * It provides commands to start (shoot) and stop the shooting motors.
     */
    public class ShootingSystem implements Subsystem {

        // Singleton instance of the ShootingSystem subsystem (initially null)
        private static ShootingSystem INSTANCE = null;

        // Telemetry for displaying motor status
        private Telemetry telemetry;

        // Motor instances representing the shooting motors
        private MotorEx shootingMotor1 = new MotorEx("sm1");
        private MotorEx shootingMotor2 = new MotorEx("sm2");

        private static double CORRECTION_DELTA = 0.00;

        // Motor group with motor2 reversed
        //private MotorGroup shooterMotors;

        // Private constructor to ensure only one instance exists
        private ShootingSystem(Telemetry telemetry) {
            this.telemetry = telemetry;
            shootingMotor1.reversed();  // Set motor2 to run in reverse
            //shooterMotors = new MotorGroup(motor2, motor1);
        }

        public void stop() {
            shootingMotor1.setPower(STOP_POWER);
            shootingMotor2.setPower(STOP_POWER);
        }

        /**
         * Gets or creates the singleton instance of the ShootingSystem subsystem.
         * @param telemetry The telemetry object to use for displaying data
         * @return The singleton instance
         */
        public static ShootingSystem getInstance(Telemetry telemetry) {
            if (INSTANCE == null) {
                INSTANCE = new ShootingSystem(telemetry);
            }
            return INSTANCE;
        }

        /**
         * Gets the existing singleton instance (use only after getInstance(telemetry) has been called)
         * @return The singleton instance
         */
        public static ShootingSystem getInstance() {
            if (INSTANCE == null) {
                throw new IllegalStateException("ShootingSystem must be initialized with telemetry first!");
            }
            return INSTANCE;
        }

        // Power levels for different shooting states
        private static final double LOW_THRESHOLD = 0.30;      // Low power shoot
        private static final double HIGH_SHOOTING_POWER = 0.99;     // High power shoot
        private static final double STOP_POWER = 0.0;     // Motors off

        private static final double SHOOTING_PWR_INCREMENT = 0.05;

        public Command stopAllSubsystems = new InstantCommand(() -> {
                shootingMotor1.setPower(STOP_POWER);
                shootingMotor2.setPower(STOP_POWER);

                // Get the object of Intake singleton class and stop that motor
                Intake.getInstance().stop();
                BallLoadingServo.getInstance().stop();

        }).requires(this);

        // Command to start the shooting motors at low power
        public Command startStop(double shootingPower) {
            return new InstantCommand(() -> {
                if (shootingMotor1.getPower() <= 0.10) {
                    shootingMotor1.setPower(shootingPower);
                    shootingMotor2.setPower(shootingPower);
                    //telemetry.addData("Shooting Mode", "THIRST");
                } else {
                    shootingMotor1.setPower(STOP_POWER);
                    shootingMotor2.setPower(STOP_POWER);
                    //telemetry.addData("Shooting Mode", "STOP");
                }
                //telemetry.update();
            }).requires(this);
        }

        public Command start(double shootingPower) {
            return new InstantCommand(() -> {
                shootingMotor1.setPower(shootingPower + CORRECTION_DELTA);
                shootingMotor2.setPower(shootingPower + CORRECTION_DELTA);
                //telemetry.addData("Shooting Mode", "THIRST");
                //telemetry.update();
            }).requires(this);
        }

        // Command to start the shooting motors at high power
        public Command increaseShootingPower = new InstantCommand(() -> {
            double newShootingPower = shootingMotor1.getPower() + SHOOTING_PWR_INCREMENT;
            if (newShootingPower >= HIGH_SHOOTING_POWER) {
                shootingMotor1.setPower(HIGH_SHOOTING_POWER);
                shootingMotor2.setPower(HIGH_SHOOTING_POWER);
                //telemetry.addData("Shooting Mode", "HIGH");
            } else {
                shootingMotor1.setPower(newShootingPower);
                shootingMotor2.setPower(newShootingPower);
                //telemetry.addData("Shooting Mode", "CUSTOM: " + String.format("%.2f", newShootingPower));
            }
            //telemetry.update();
        }).requires(this);

        public Command decreaseShootingPower = new InstantCommand(() -> {
            double newShootingPower = shootingMotor1.getPower() - SHOOTING_PWR_INCREMENT;
            if (newShootingPower <= LOW_THRESHOLD) {
                shootingMotor1.setPower(LOW_THRESHOLD);
                shootingMotor2.setPower(LOW_THRESHOLD);
                //telemetry.addData("Shooting Mode", "LOW_THRESHOLD");
            } else {
                shootingMotor1.setPower(newShootingPower);
                shootingMotor2.setPower(newShootingPower);
                //telemetry.addData("Shooting Mode", "CUSTOM: " + String.format("%.2f", newShootingPower));
            }
            //telemetry.update();
        }).requires(this);

        public void setPower(double power) {
            shootingMotor1.setPower(power);
            shootingMotor2.setPower(power);
        }

        // Command to stop the shooting motors
        public Command stop = new InstantCommand(() -> setPower(STOP_POWER)).requires(this);

        /**
         * The periodic method is called repeatedly while the robot is running.
         * Displays telemetry data about the shooting motors status.
         */
        @Override
        public void periodic() {
            telemetry.addData("<===== Shooting Speed Correction =====>", "");
            telemetry.addData("CORRECTION SHOOTING POWER: ", "%.3f", CORRECTION_DELTA);

            telemetry.addData("<=====Shooting System=====>","");
            telemetry.addData("Shooter 1", "Pwr: %.2f | Pos: %.2f | Vel: %.2f",
                    shootingMotor1.getPower(), shootingMotor1.getCurrentPosition(), shootingMotor1.getVelocity());
            telemetry.addData("Shooter 2", "Pwr: %.2f | Pos: %.2f | Vel: %.2f",
                    shootingMotor2.getPower(), shootingMotor2.getCurrentPosition(), shootingMotor2.getVelocity());
            telemetry.update();
        }

        public Command INCREASE_DELTA = new InstantCommand(() -> {
            if (CORRECTION_DELTA <= 0.05) {
                CORRECTION_DELTA = CORRECTION_DELTA + 0.01;
            }
            telemetry.addData("CORRECTION SHOOTING POWER: ", "%.3f", CORRECTION_DELTA);
        }).requires(this);
        public Command DECREASE_DELTA = new InstantCommand(() -> {
            if (CORRECTION_DELTA > -0.05) {
                CORRECTION_DELTA = CORRECTION_DELTA - 0.01;
            }
            telemetry.addData("CORRECTION SHOOTING POWER: ", "%.3f", CORRECTION_DELTA);
        }).requires(this);

    }