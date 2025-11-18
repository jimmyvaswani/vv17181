package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Blue - Front Zone - Reload NEW", group = "AUTON")
public class BlueFrontReload extends NextFTCOpMode {
    // --- Subsystem Instances (Initialized in onInit)
    private Intake intake;
    private ShootingSystem shooter;
    private ShootingDirectionServo shootingDirection;
    private BallLoadingServo ballLoader;

    // --- Pose Definitions (Coordinates for autonomous movement)
    private final Pose detourPose = new Pose(90, 70, Math.toRadians(181));
    private final Pose detourPose2 = new Pose(100, 70, Math.toRadians(181));
    private final Pose startPose = new Pose(20, 120, Math.toRadians(310));
    private final Pose scorePose = new Pose(35, 95, Math.toRadians(330));


    // Assumed pick-up locations (using similar logic to original)
    private static final Pose pickUpOneStage = new Pose(50, 78, Math.toRadians(181));
    private static final Pose pickUpOne = new Pose(25, 78, Math.toRadians(181));
    private static final Pose pickUpTwoStage = new Pose(50, 78, Math.toRadians(181));
    private static final Pose pickUpTwo = new Pose(25, 78, Math.toRadians(181));
    private static final Pose pickUpThreeStage = new Pose(50, 58, Math.toRadians(181));
    private static final Pose pickUpThree = new Pose(25, 58, Math.toRadians(181));

    private TelemetryManager panelsTelemetry;

    // --- Path Definitions (Initialized in buildPaths)
    private Path scorePreload;
    private PathChain moveToPickUpOne;
    private PathChain doPickUpOne;
    private PathChain scorePickUpOne;
    private PathChain detourOne, detourTwo;

    // --- Autonomous Routine Parameters
    private static final double POSITION_A = 0.88;          // Shooting Direction Servo position angled for the second shooting point
    private static final double SHOOTING_POWER_A = 0.48;    // Low power shoot

    // Motion tuning
    private static final double SPEED_TO_SCORE = 1.0;
    private static final double SPEED_DETOUR = 0.60;
    private static final double SPEED_PICKUP_APPROACH = 0.60;
    private static final double SPEED_PICKUP_PASS = 0.40;

    // Timing tuning
    private static final double SHOOT_SPINUP_TIME = 0.5;
    private static final double DELAY_BEFORE_INTAKE = 0.5;
    private static final double SHOOT_RUN_TIME = 4.0;

    public BlueFrontReload() {
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
        shootingDirection.setPosition(0.86);
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(

                // 1. Drive to preload scoring position
                new FollowPath(scorePreload, true, SPEED_TO_SCORE),
                // 2. Shoot preload
                runShootingRoutine(),
                new TurnBy(Angle.fromDeg(30))
/*
                // --- Reload Sequence 1 ---

                // 3. Drive toward pickup 1 staging
                new FollowPath(detourOne, true, SPEED_DETOUR),
                new FollowPath(moveToPickUpOne, true, SPEED_PICKUP_APPROACH),

                // 4. Run intake and do pickup 1
                new Delay(DELAY_BEFORE_INTAKE), // brief settle time before intake
                intake.START,
                new FollowPath(doPickUpOne, true, SPEED_PICKUP_PASS),
                intake.STOP,

                // 5. Return to score and shoot pickup 1
                new FollowPath(detourTwo, true, SPEED_DETOUR),
                new FollowPath(scorePickUpOne, true, 0.60),

                runShootingRoutine() */
        );
    }

    private Command runShootingRoutine() {
        return new SequentialGroup(
                shooter.start(SHOOTING_POWER_A),
                new Delay(SHOOT_SPINUP_TIME), //For shooting motor to get to the right speed
                shootingDirection.PositionForSpecificDistance(POSITION_A),
                ballLoader.runBackward(),
                intake.START,
                new Delay(SHOOT_RUN_TIME), //Running the systems for 4 seconds, enough time for all three balls to shoot.
                shooter.stopAllSubsystems
        );
    }

    @Override
    public void onStartButtonPressed() {
        log("AUTO", "onStartButtonPressed: setting start pose and building paths");
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().setPose(startPose);
        buildPaths();
        log("AUTO", "Paths built, scheduling autonomous routine");
        autonomousRoutine().schedule();
        log("AUTO", "Autonomous routine scheduled");
    }

    private void buildPaths() {
        // Path from Start to Score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path from Score to PickUp 1 Staging
        detourOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, detourPose) )
                .setLinearHeadingInterpolation(scorePose.getHeading(), detourPose.getHeading()).build();

        // Path from Score to PickUp 1 Staging
        moveToPickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(detourPose, pickUpOneStage) )
                .setLinearHeadingInterpolation(detourPose.getHeading(), pickUpOneStage.getHeading()).build();

        // Path from PickUp 1 Staging to PickUp 1
        doPickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpOneStage, pickUpOne))
                .setLinearHeadingInterpolation(pickUpOneStage.getHeading(), pickUpOne.getHeading()).build();

        // Path from Score to PickUp 1 Staging
        detourTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickUpOne, detourPose2) )
                .setLinearHeadingInterpolation(pickUpOne.getHeading(), detourPose2.getHeading()).build();

        // Path from PickUp 1 back to Score
        scorePickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(detourPose2, scorePose))
                .setLinearHeadingInterpolation(detourPose2.getHeading(), scorePose.getHeading()).build();

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
