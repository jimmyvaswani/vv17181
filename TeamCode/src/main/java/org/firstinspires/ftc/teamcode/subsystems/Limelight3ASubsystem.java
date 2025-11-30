package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Limelight3ASubsystem implements Subsystem {

    private static Limelight3ASubsystem INSTANCE;
    private Telemetry telemetry;

    private Limelight3A vvLimelight;
    LLResult llResult = null;
    YawPitchRollAngles orientation = null;
    Pose3D robotPose = null;
    private IMUEx imu = null;
    private double distance = 0.0;
    // scale calculate using mycurvefit.com
    // reference youtube video by Brogan M. Patt (how to measure distance with April tags (limelight 3A)"
    //private final double scale = 29024.85; // Value derived using mycurvefit.com
    private final double scale = 180.51; // Anuj/Team: Reverse engineering scale value derived using scale = knownDistance*SQRT(ta)
    private final int currentPipeline = 2;
    public char identifiedShootingPosition = '0'; //Zero is default

    // --- Tuning constants for position bands ---
    // How close to image center (in degrees of tx) we require the tag to be
    // Tx tolerance bands per position (MIN–MAX degrees left/right)
// TODO: tune these ranges on the field for each position
    private static final double A_TX_MIN_DEG = -5.0;   // A: very close to center
    private static final double A_TX_MAX_DEG =  5.0;

    private static final double B_TX_MIN_DEG = -5.0;   // B: slightly wider
    private static final double B_TX_MAX_DEG =  5.0;

    private static final double X_TX_MIN_DEG = -4.0;   // X: mid-wide window
    private static final double X_TX_MAX_DEG =  4.0;

    private static final double Y_TX_MIN_DEG = -3.0;   // Y: widest tracked window
    private static final double Y_TX_MAX_DEG =  3.0;

    // Distance bands in INCHES (use MIN–MAX ranges)
    private static final double A_MIN_DISTANCE_IN = 22.0;
    private static final double A_MAX_DISTANCE_IN = 26.0;

    private static final double B_MIN_DISTANCE_IN = 43.0;
    private static final double B_MAX_DISTANCE_IN = 49.0;

    private static final double X_MIN_DISTANCE_IN = 68.0;
    private static final double X_MAX_DISTANCE_IN = 76.0;

    private static final double Y_MIN_DISTANCE_IN = 126.0;
    private static final double Y_MAX_DISTANCE_IN = 135.0;

    private Limelight3ASubsystem(HardwareMap hardwareMap, Telemetry telemetry, IMUEx imu) {
        this.telemetry = telemetry;
        this.imu = imu;
        vvLimelight = hardwareMap.get(Limelight3A.class, "limelight");
        vvLimelight.pipelineSwitch(currentPipeline);
        vvLimelight.start();
    }

    public static Limelight3ASubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry, IMUEx imu) {
        if (INSTANCE == null) {
            INSTANCE = new Limelight3ASubsystem(hardwareMap, telemetry, imu);
        }
        return INSTANCE;
    }

    public static Limelight3ASubsystem getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("You must set up the servo first using telemetry!");
        }
        return INSTANCE;
    }

/*    @Override
    public void initialize() {
        Subsystem.super.initialize();
        vvLimelight.pipelineSwitch(currentPipeline);
        vvLimelight.start();
        telemetry.addData("<===== Limelight Camera Started =====>", "");
    }*/

    @Override
    public void periodic() {
        telemetry.addData("<=== Limelight Camera ===>", " (Pipeline: %d)", currentPipeline);
        orientation = imu.getImu().getRobotYawPitchRollAngles();
        vvLimelight.updateRobotOrientation(orientation.getYaw());
        llResult = vvLimelight.getLatestResult();
        identifiedShootingPosition = '0'; //Always start with the default position zero

        //Logic help to debug bad/no results
        /*if (llResult == null) {
            telemetry.addData("Limelight Result", "NULL - No data from camera");
        } else if (!llResult.isValid()) {
            telemetry.addData("Limelight Result", "Invalid - No targets detected");
        } else*/
        if(llResult != null && llResult.isValid()) {

            if (llResult.getTa()>0) {
                // reference youtube video by Brogan M. Patt (how to measure distance with April tags (limelight 3A)"
                distance = scale/Math.sqrt(llResult.getTa()); //will get values in cm (because scale was calculated with distance in CMs 60,120,180,240)
                distance = distance * 0.393701; //converted to inches
            }
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

            //Identify the very first April Tag and print it.
            if (fiducials!=null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int tagId = fiducial.getFiducialId();  // <-- This is the AprilTag number
                    telemetry.addData("AprilTag ID", tagId);
                    break;
                }
            }

            robotPose = llResult.getBotpose_MT2(); // llResult.getBotpose() if need megaTag1 data only. This is configured during the pipeline (MT2 is the yellow bot)

            // Classify this frame into a discrete shooting position based on tx + distance
            identifiedShootingPosition = classifyShootingPosition(llResult.getTx(), distance);

            telemetry.addData("Limelight", "Tx: %.3f | Ty: %.3f | Ta: %.3f | Yaw: %.3f | Distance: %.3f | Pos: %s",
                    llResult.getTx(), llResult.getTy(), llResult.getTa(), robotPose.getOrientation().getYaw(), distance, identifiedShootingPosition);
        }
    }

    /**
     * Classify the robot's current location relative to the AprilTag into
     * one of the discrete positions A, B, X, Y, or O (other).
     *
     * A is the closest band, then B, X, Y in ascending order by distance.
     * Tag must also be near the image center (|tx| <= TX_TOLERANCE_DEG)
     * or it will be classified as 'O'.
     */
    private char classifyShootingPosition(double tx, double distanceInches) {
        char position = '0'; //Zero is default

        // A: closest band + tightest tx window
        if (distanceInches >= A_MIN_DISTANCE_IN && distanceInches <= A_MAX_DISTANCE_IN &&
                tx >= A_TX_MIN_DEG && tx <= A_TX_MAX_DEG) {
            position = 'A';

            // B: next distance band + its own tx window
        } else if (distanceInches >= B_MIN_DISTANCE_IN && distanceInches <= B_MAX_DISTANCE_IN &&
                tx >= B_TX_MIN_DEG && tx <= B_TX_MAX_DEG) {
            position = 'B';

            // X: next band
        } else if (distanceInches >= X_MIN_DISTANCE_IN && distanceInches <= X_MAX_DISTANCE_IN &&
                tx >= X_TX_MIN_DEG && tx <= X_TX_MAX_DEG) {
            position = 'X';

            // Y: farthest band we still care about
        } else if (distanceInches >= Y_MIN_DISTANCE_IN && distanceInches <= Y_MAX_DISTANCE_IN &&
                tx >= Y_TX_MIN_DEG && tx <= Y_TX_MAX_DEG) {
            position = 'Y';
        }

        return position;
    }

    // getters: hasTarget(), getTx(), getTy(), getTa(), getRobotPose(), etc.
    public double getDistanceFromTag() {
        return distance;
    }

    public char getIdentifiedShootingPosition() {
        return identifiedShootingPosition;
    }
}
