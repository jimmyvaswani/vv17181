/* This is updated code as of 10/19/2025 */
package org.firstinspires.ftc.teamcode.teleop;

// These are names we made for each wheel motor in another file (ChassisConstants)
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// These are the "subsystems" — little robot parts that do special jobs
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.Light;


import org.firstinspires.ftc.robotcore.external.Telemetry;

// These come from the NextFTC library
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Robot Centric TeleOp")
public class RobotCentricTeleOp extends NextFTCOpMode {

    //Added this code to fix the speed on robot navigation
    private static final double STRAFE_SPEED_MULTIPLIER = 0.7; // <--- ADD THIS LINE HERE
    // These are the robot's different systems (we'll use them later)
    private ShootingSystem shootingSystem;
    private ShootingDirectionServo shootingDirectionServo;
    private BallLoadingServo ballLoadingServo;
    private Intake intakeSystem;
    private Light light;

    private final double POSITION_CLOSEST = 0.86; // Shooting Direction Servo position angled for the closer shooting point
    private final double POSITION_SECOND = 0.92; // Shooting Direction Servo position angled for the closer shooting point
    private final double POSITION_THIRD = 0.96; // Shooting Direction Servo position angled for the closer shooting point
    private final double POSITION_FOURTH = 0.96; // Shooting Direction Servo position angled for the closer shooting point

    private static final double FIRST_SHOOTING_POWER = 0.40;      // Low power shoot
    private static final double SECOND_SHOOTING_POWER = 0.50;      // Low power shoot
    private static final double THIRD_SHOOTING_POWER = 0.60;      // Medium power shoot
    private static final double FOURTH_SHOOTING_POWER = 0.75;      // Medium power shoot


    protected final IMUEx imu = new IMUEx("imu", Direction.DOWN, Direction.FORWARD).zeroed();

    // This is the "constructor" — runs once when the program starts loading
    public RobotCentricTeleOp() {

        // We connect our subsystems to the robot and give them access to telemetry (data shown on driver station)
        shootingSystem = ShootingSystem.getInstance(telemetry);
        intakeSystem = Intake.getInstance(telemetry);
        shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        ballLoadingServo = BallLoadingServo.getInstance(telemetry);

        // FIX: Get the singleton instance instead of creating new Light()
        light = Light.INSTANCE;

        // Here we "add" all these subsystems so NextFTC can manage and update them automatically
        addComponents(
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(intakeSystem),
                new SubsystemComponent(shootingDirectionServo),
                new SubsystemComponent(ballLoadingServo),
                new SubsystemComponent(light),
                BulkReadComponent.INSTANCE,   // reads all sensors at once for faster updates
                BindingsComponent.INSTANCE    // helps connect buttons on the gamepads to commands
        );
    }

    // These are our 4 drive motors (the robot's wheels)
    // The MotorEx class is like a smarter motor object from the NextFTC library
    private final MotorEx frontLeftMotor = new MotorEx(LEFT_FRONT_MOTOR_NAME).reversed();
    private final MotorEx frontRightMotor = new MotorEx(RIGHT_FRONT_MOTOR_NAME);
    private final MotorEx backLeftMotor = new MotorEx(LEFT_REAR_MOTOR_NAME).reversed();
    private final MotorEx backRightMotor = new MotorEx(RIGHT_REAR_MOTOR_NAME);

    // This method runs when the driver presses the START button on the Driver Station
    @Override
    public void onStartButtonPressed() {

        light.setPattern(Light.SOLID_GREEN_POSITION).schedule();
        // This tells the robot how to drive using mecanum wheels (which move in all directions)
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().map(y -> y * -1.0 * STRAFE_SPEED_MULTIPLIER),
                //Gamepads.gamepad1().leftStickX(),  // strafe left/right
                Gamepads.gamepad1().leftStickX().map(x -> x * 1.0 * STRAFE_SPEED_MULTIPLIER),
                Gamepads.gamepad1().rightStickX().map(x -> x * 1.0 * STRAFE_SPEED_MULTIPLIER)
                );

        // "Schedule" means start running that drive command
        driverControlled.schedule();

        /* 🎮 GAMEPAD 2 CONTROLS (the second controller) */

        // Left bumper → Reverse the Intake system and loading servo
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(
                new SequentialGroup(
                        intakeSystem.reverse,
                        ballLoadingServo.runForward(),
                        new Delay(0.25), //Running the reverse operation only for 1/4 of a seconds
                        ballLoadingServo.stopContinuous(),
                        intakeSystem.STOP
                )
        );

        // Right bumper → start/stop the Intake system
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(intakeSystem.startStop);

        // Intake System Controls on Gamepad 2 as before
        Gamepads.gamepad2().a().whenBecomesTrue(
                new SequentialGroup(
                        shootingSystem.start(FIRST_SHOOTING_POWER),
                        shootingDirectionServo.PositionForSpecificDistance(POSITION_CLOSEST),
                        ballLoadingServo.runBackward(),
                        intakeSystem.START,
                        new Delay(5.0), //Running the systems for 5 seconds
                        shootingSystem.stopAllSubsystems
                )
        );
        Gamepads.gamepad2().b().whenBecomesTrue(
                new SequentialGroup(
                        shootingSystem.start(SECOND_SHOOTING_POWER),
                        shootingDirectionServo.PositionForSpecificDistance(POSITION_SECOND),
                        ballLoadingServo.runBackward(),
                        intakeSystem.START,
                        new Delay(5.0), //Running the systems for 5 seconds
                        shootingSystem.stopAllSubsystems
                )
        );
        Gamepads.gamepad2().x().whenBecomesTrue(
                new SequentialGroup(
                        shootingSystem.start(THIRD_SHOOTING_POWER),
                        shootingDirectionServo.PositionForSpecificDistance(POSITION_THIRD),
                        ballLoadingServo.runBackward(),
                        intakeSystem.START,
                        new Delay(5.0), //Running the systems for 5 seconds
                        shootingSystem.stopAllSubsystems
                )
        );
        Gamepads.gamepad2().y().whenBecomesTrue(
                new SequentialGroup(
                        shootingSystem.start(FOURTH_SHOOTING_POWER),
                        shootingDirectionServo.PositionForSpecificDistance(POSITION_FOURTH),
                        ballLoadingServo.runBackward(),
                        intakeSystem.START,
                        new Delay(5.0), //Running the systems for 5 seconds
                        shootingSystem.stopAllSubsystems
                )
        );

        Gamepads.gamepad2().rightTrigger().greaterThan(0.25).whenBecomesTrue(shootingSystem.increaseShootingPower);
        Gamepads.gamepad2().leftTrigger().greaterThan(0.25).whenBecomesTrue(shootingSystem.decreaseShootingPower);

        // Back button → stop everything in the shooting system
        Gamepads.gamepad2().back().whenBecomesTrue(shootingSystem.stopAllSubsystems);

        // D-Pad Left → run the ball loading servo backward (to load balls)
        //Gamepads.gamepad2().dpadLeft().whenBecomesTrue(ballLoadingServo.runBackward());

        // D-Pad Right → stop the ball loading servo
        //Gamepads.gamepad2().dpadRight().whenBecomesTrue(ballLoadingServo.stopContinuous());
        //Gamepads.gamepad2().dpadRight().whenBecomesTrue(ballLoadingServo.REVERSEANDSTOP());
//New Update
        // D-Pad Up → aim the shooter down
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(shootingDirectionServo.downShootingServo);

        // D-Pad Down → aim the shooter up
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(shootingDirectionServo.upShootingServo);
    }

}