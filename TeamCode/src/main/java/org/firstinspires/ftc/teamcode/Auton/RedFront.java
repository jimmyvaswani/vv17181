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
//import org.firstinspires.ftc.teamcode.subsystems.Launcher;
//import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red - Front Zone")
public class RedFront extends NextFTCOpMode {
    private final Pose startPose = new Pose(114, 136, Math.toRadians(270));
    private final Pose scorePose = new Pose(78, 78, Math.toRadians(230));
    private final Pose endPose = new Pose(96, 48, Math.toRadians(180));

    private TelemetryManager panelsTelemetry;

    private Path scorePreload;
    private PathChain leave;

    public RedFront() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                //new SubsystemComponent(LauncherSubsystem.INSTANCE, IntakeSubsystem.INSTANCE),
                new SubsystemComponent(ShootingSystem.getInstance(telemetry)),
                new SubsystemComponent(Intake.getInstance(telemetry)),
                BulkReadComponent.INSTANCE
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                //Lift.INSTANCE.preLoad,
                new FollowPath(scorePreload, true, 0.75),
                //LauncherSubsystem.INSTANCE.launchTwo,
                new ParallelGroup(
                        Intake.getInstance().START
                        //, new FollowPath(leave, true, 1.00)
                ),
                new Delay(0.5),
                Intake.getInstance().STOP
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

        //leave = PedroComponent.follower().pathBuilder()
        //        .addPath(new BezierLine(scorePose, endPose))
        //        .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading()).build();;
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