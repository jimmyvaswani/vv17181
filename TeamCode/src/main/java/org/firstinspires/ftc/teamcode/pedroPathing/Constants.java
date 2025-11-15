package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.ChassisConstants.*;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ChassisConstants;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.57)
            .forwardZeroPowerAcceleration(-26.66)
            .lateralZeroPowerAcceleration(-58.35)

            // Keeping original values as is.
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0009)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.01, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.2, 0.01));


    /* Recommended values for the heavier, lower-accelerating robot, values pulled from google AI
        .useSecondaryTranslationalPIDF(false)
        .useSecondaryHeadingPIDF(false)
        .useSecondaryDrivePIDF(false)
        .centripetalScaling(0.001) // Slight increase for stability
        .translationalPIDFCoefficients(new PIDFCoefficients(0.35, 0, 0.02, 0.02)) // Increased Kp for responsiveness
        .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.05, 0.03)) // Using the well-tuned values from File 1
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.8, 0.01)); // Increased Kf (0.8) for higher speed target
*/
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName(LEFT_FRONT_MOTOR_NAME)
            .leftRearMotorName(LEFT_REAR_MOTOR_NAME)
            .rightFrontMotorName(RIGHT_FRONT_MOTOR_NAME)
            .rightRearMotorName(RIGHT_REAR_MOTOR_NAME)
            .leftFrontMotorDirection(LEFT_FRONT_MOTOR_DIRECTION)
            .leftRearMotorDirection(LEFT_REAR_MOTOR_DIRECTION)
            .rightFrontMotorDirection(RIGHT_FRONT_MOTOR_DIRECTION)
            .rightRearMotorDirection(RIGHT_REAR_MOTOR_DIRECTION)
            .xVelocity(76.11)  //Forward velocity
            .yVelocity(61.17); //Strafe Velocity

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(5.51181)
            .strafePodX(-5.19685)
            .forwardEncoder_HardwareMapName(RIGHT_FRONT_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(LEFT_FRONT_MOTOR_NAME)
            .strafeEncoderDirection(Encoder.REVERSE) //comment out
            .forwardEncoderDirection(Encoder.FORWARD) //added new
            .forwardTicksToInches((0.944882 * 2 * Math.PI)/2000)  // .forwardTicksToInches(0.00294)
            .strafeTicksToInches((0.944882 * 2 * Math.PI)/2000) // .strafeTicksToInches(0.00293)

            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, //0.995
            100, //500
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

