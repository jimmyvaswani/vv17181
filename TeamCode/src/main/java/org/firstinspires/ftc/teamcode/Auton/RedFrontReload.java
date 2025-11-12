package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red - Front Zone - Reload NEW", group = "experimental")
public class RedFrontReload extends NextFTCOpMode {
    // --- Subsystem Instances (Initialized in onInit)
    private Intake intake;
    private ShootingSystem shooter;
    private ShootingDirectionServo shootingDirection;
    private BallLoadingServo ballLoader;

    // --- Pose Definitions (Coordinates for autonomous movement)
    private final Pose startPose = new Pose(114, 136, Math.toRadians(270));
    private final Pose scorePose = new Pose(85, 80, Math.toRadians(225));
    private final Pose endPose = new Pose(96, 48, Math.toRadians(180));

    // Assumed pick-up locations (using similar logic to original)
    private static final Pose pickUpOneStage = new Pose(85, 98, Math.toRadians(0));
    private static final Pose pickUpOne = new Pose(110, 98, Math.toRadians(0));
    private static final Pose pickUpTwoStage = new Pose(85, 74, Math.toRadians(0));
    private static final Pose pickUpTwo = new Pose(110, 74, Math.toRadians(0));

    private TelemetryManager panelsTelemetry;

    // --- Path Definitions (Initialized in buildPaths)
    private Path scorePreload;
    private PathChain moveToPickUpOne;
    private PathChain doPickUpOne;
    private PathChain scorePickUpOne;
    private PathChain moveToPickUpTwo;
    private PathChain doPickUpTwo;
    private PathChain scorePickUpTwo;
    private PathChain leave;

    // --- Autonomous Routine Parameters
    private static final double SHOOTING_POWER = 0.99; // Using HIGH_SHOOTING_POWER from ShootingSystem
    private static final double LOADER_POWER = 1.0; // Full power for the ball loader
    private static final double WARMUP_TIME = 1.0; // Time for the shooter to spin up

    public RedFrontReload() {
        // Only PedroComponent and BulkReadComponent are initialized here.
        // The Subsystems requiring Telemetry are initialized in onInit/onStart.
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        super.onInit();
        // Initialize Subsystems requiring Telemetry inside an OpMode method
        intake = Intake.getInstance(telemetry);
        shooter = ShootingSystem.getInstance(telemetry);
        shootingDirection = ShootingDirectionServo.getInstance(telemetry);
        ballLoader = BallLoadingServo.getInstance(telemetry);

        // Add Subsystems to the NextFTC component manager
        addComponents(
                new SubsystemComponent(intake, shooter, shootingDirection, ballLoader)
        );

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Set an initial position for the shooting direction servo (assuming 0.85 is a safe starting position)
        shootingDirection.setPosition(0.85);
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                // 1. Initial Setup: Start shooter and set aiming angle
                new ParallelGroup(
                        shooter.start(SHOOTING_POWER), // Start shooter spin-up
                        shootingDirection.PositionForSpecificDistance(0.85) // Set angle for preload shot
                ),

                // 2. Move to Scoring Position
                new FollowPath(scorePreload, true, 0.5),

                // 3. Score Preload
                new SequentialGroup(
                        new Delay(WARMUP_TIME), // Wait for shooter to spin up
                        ballLoader.runForward(LOADER_POWER), // Load the ball into the shooter
                        new Delay(0.5), // Scoring delay
                        ballLoader.stopContinuous() // Stop the loader
                ),

                // --- Reload Sequence 1 ---

                // 4. Move to Pick Up 1 (Parallel: Stop shooter, start intake, move)
                new ParallelGroup(
                        shooter.stop, // Stop the shooter
                        intake.START, // Start the intake motor (Gate logic is missing/internal)
                        new FollowPath(moveToPickUpOne, false, 1.00)
                ),

                // 5. Execute Pick Up 1
                new FollowPath(doPickUpOne, true, 1.00),
                intake.STOP, // Stop the intake motor (assuming item is picked up)

                // 6. Score Pick Up 1 (Parallel: Start shooter, move back to score pose)
                new ParallelGroup(
                        shooter.start(SHOOTING_POWER), // Start shooter spin-up
                        new FollowPath(scorePickUpOne, true, 0.7)
                ),

                // 7. Fire Pick Up 1
                new SequentialGroup(
                        new Delay(WARMUP_TIME), // Wait for shooter to spin up
                        ballLoader.runForward(LOADER_POWER), // Load the ball
                        new Delay(0.5), // Scoring delay
                        ballLoader.stopContinuous()
                ),

                // --- Reload Sequence 2 (Simplified version) ---

                // 8. Move to Pick Up 2 (Parallel: Stop shooter, start intake, move)
                new ParallelGroup(
                        shooter.stop,
                        intake.START,
                        new FollowPath(moveToPickUpTwo, false, 1.00)
                ),

                // 9. Execute Pick Up 2
                new FollowPath(doPickUpTwo, true, 1.00),
                intake.STOP,

                // 10. Score Pick Up 2 (Parallel: Start shooter, move back to score pose)
                new ParallelGroup(
                        shooter.start(SHOOTING_POWER),
                        new FollowPath(scorePickUpTwo, true, 0.7)
                ),

                // 11. Fire Pick Up 2
                new SequentialGroup(
                        new Delay(WARMUP_TIME),
                        ballLoader.runForward(LOADER_POWER),
                        new Delay(0.5),
                        ballLoader.stopContinuous()
                ),

                // 12. Park
                new ParallelGroup(
                        shooter.stop,
                        new FollowPath(leave, true, 1.00)
                )

        );
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().setPose(startPose);
        buildPaths();
        autonomousRoutine().schedule();
    }

    private void buildPaths() {
        // Path from Start to Score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path from Score to PickUp 1 Staging
        moveToPickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, pickUpOneStage))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpOneStage.getHeading()).build();

        // Path from PickUp 1 Staging to PickUp 1
        doPickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpOneStage, pickUpOne))
                .setLinearHeadingInterpolation(pickUpOneStage.getHeading(), pickUpOne.getHeading()).build();

        // Path from PickUp 1 back to Score
        scorePickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpOne, scorePose))
                .setLinearHeadingInterpolation(pickUpOne.getHeading(), scorePose.getHeading()).build();

        // Path from Score to PickUp 2 Staging
        moveToPickUpTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, pickUpTwoStage))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpTwoStage.getHeading()).build();

        // Path from PickUp 2 Staging to PickUp 2
        doPickUpTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpTwoStage, pickUpTwo))
                .setLinearHeadingInterpolation(pickUpTwoStage.getHeading(), pickUpTwo.getHeading()).build();

        // Path from PickUp 2 back to Score
        scorePickUpTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpTwo, scorePose))
                .setLinearHeadingInterpolation(pickUpTwo.getHeading(), scorePose.getHeading()).build();

        // Path from Score to End/Park
        leave = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading()).build();
    }

    // The log method from the original file (optional but included for completeness)
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }
}