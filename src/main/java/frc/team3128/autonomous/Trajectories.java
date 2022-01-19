package frc.team3128.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import frc.team3128.Constants;

/**
 * 
 * Store trajectories for autonomous. Edit points here. 
 * This class may be used as a backup for PathWeaver trajectories.
 */
public class Trajectories {

    private static double xOffsetInches = 50;
    private static double yOffsetInches = 50;

    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV, Constants.DriveConstants.kA),
            Constants.DriveConstants.DRIVE_KINEMATICS, Constants.DriveConstants.MAX_DRIVE_VOLTAGE);

    private static final TrajectoryConfig forwardTrajConfig = new TrajectoryConfig(Constants.DriveConstants.MAX_DRIVE_VELOCITY,
            Constants.DriveConstants.MAX_DRIVE_ACCELERATION)
            .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint)
            .setReversed(false);

    private static final TrajectoryConfig backwardsTrajConfig = new TrajectoryConfig(Constants.DriveConstants.MAX_DRIVE_VELOCITY,
            Constants.DriveConstants.MAX_DRIVE_ACCELERATION)
            .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    public static Trajectory trajectorySimple = TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(0 + xOffsetInches), Units.inchesToMeters(0 + yOffsetInches), new Rotation2d(0)),
        List.of(
        new Translation2d(Units.inchesToMeters(50 + xOffsetInches), Units.inchesToMeters(0 + yOffsetInches)),  
        new Translation2d(Units.inchesToMeters(70 + xOffsetInches), Units.inchesToMeters(-25 + yOffsetInches))                                                                                                       
        ),
        new Pose2d(Units.inchesToMeters(50 + xOffsetInches), Units.inchesToMeters(-50 + yOffsetInches), new Rotation2d(-Math.PI)),
        forwardTrajConfig);

    public static Trajectory trajectoryLessSimple = TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(0 + xOffsetInches), Units.inchesToMeters(0 + yOffsetInches), new Rotation2d(0)),
        List.of(
        new Translation2d(Units.inchesToMeters(30 + xOffsetInches), Units.inchesToMeters(20 + yOffsetInches)),
        new Translation2d(Units.inchesToMeters(60 + xOffsetInches), Units.inchesToMeters(-20 + yOffsetInches))                                                          
        ),
        new Pose2d(Units.inchesToMeters(90 + xOffsetInches), Units.inchesToMeters(0 + yOffsetInches), new Rotation2d(0)),
        forwardTrajConfig);
}
