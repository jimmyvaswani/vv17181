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
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red - Front Zone - Reload", group = "experimental")
public class RedFront extends NextFTCOpMode {
    private final Pose startPose = new Pose(114, 136, Math.toRadians(270));
    private final Pose scorePose = new Pose(85, 80, Math.toRadians(225));
    private final Pose endPose = new Pose(96, 48, Math.toRadians(180));

    private static final Pose pickUpOneStage = new Pose(85, 98, Math.toRadians(0));
    private static final Pose pickUpOne= new Pose(110, 98, Math.toRadians(0));
    private static final Pose pickUpTwoStage = new Pose(85, 74, Math.toRadians(0));
    private static final Pose pickUpTwo= new Pose(110, 74, Math.toRadians(0));

    private TelemetryManager panelsTelemetry;

    private Path scorePreload;

    private PathChain moveToPickUpOne;

    private PathChain doPickUpOne;

    private PathChain scorePickUpOne;

    private PathChain moveToPickUpTwo;

    private PathChain doPickUpTwo;

    private PathChain scorePickUpTwo;
    private PathChain leave;

    public RedFront() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                //new SubsystemComponent(LauncherSubsystem.INSTANCE, IntakeSubsystem.INSTANCE),
                new SubsystemComponent(Intake.getInstance(telemetry)),
                new SubsystemComponent(ShootingSystem.getInstance(telemetry)),
                BulkReadComponent.INSTANCE
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        //Lift.INSTANCE.preLoad,
                        ShootingSystem.getInstance().start(0.5)
                ),
                new FollowPath(scorePreload, true, 0.5),
                ShootingSystem.getInstance().start(0.5),
                new Delay(0.2),
                new ParallelGroup(
                        //Launcher.INSTANCE.stop,
                        Intake.getInstance().START,
                        new FollowPath(moveToPickUpOne, false, 1.00)
                ),
                new FollowPath(doPickUpOne, true, 1.00),
                Intake.getInstance().STOP,
                new ParallelGroup(
                        //Launcher.INSTANCE.start,
                        new FollowPath(scorePickUpOne, true, 0.7)
                ),
                ShootingSystem.getInstance().start(0.5),
                new ParallelGroup(
                        //Launcher.INSTANCE.stop,
                        Intake.getInstance().START,
                        new FollowPath(moveToPickUpTwo, false, 1.00)
                ),
                new FollowPath(doPickUpTwo, true, 1.00),
                new Delay(0.2),
                Intake.getInstance().STOP,
                new ParallelGroup(
                        //Launcher.INSTANCE.start,
                        new FollowPath(scorePickUpTwo, true, 0.7)
                ),
                ShootingSystem.getInstance().start(0.5),
                new ParallelGroup(
                        //Launcher.INSTANCE.stop,
                        new FollowPath(leave, true, 1.00)
                )

        );
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().setPose(startPose);
        buildPaths();
        //Launcher.setPowerFactor(.68);
        autonomousRoutine().schedule();
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        moveToPickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, pickUpOneStage))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpOneStage.getHeading()).build();

        doPickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpOneStage, pickUpOne))
                .setLinearHeadingInterpolation(pickUpOneStage.getHeading(), pickUpOne.getHeading()).build();

        scorePickUpOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpOne, scorePose))
                .setLinearHeadingInterpolation(pickUpOne.getHeading(), scorePose.getHeading()).build();

        moveToPickUpTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, pickUpTwoStage))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpTwoStage.getHeading()).build();

        doPickUpTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpTwoStage, pickUpTwo))
                .setLinearHeadingInterpolation(pickUpTwoStage.getHeading(), pickUpTwo.getHeading()).build();

        scorePickUpTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpTwo, scorePose))
                .setLinearHeadingInterpolation(pickUpTwo.getHeading(), scorePose.getHeading()).build();

        leave = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading()).build();;
    }


    @Override
    public void onInit() {
        super.onInit();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

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