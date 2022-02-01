package frc.team3128;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.commands.ArcadeDrive;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.subsystems.NAR_Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private NAR_Drivetrain m_drive;
    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private String[] trajectoryJson = {"paths/4_Ball_i.wpilib.json", "paths/4_Ball_ii.wpilib.json"};
    private Trajectory[] trajectories = new Trajectory[trajectoryJson.length ];
    private Command auto;

    private boolean DEBUG = true;

    public RobotContainer() {
        m_drive = NAR_Drivetrain.getInstance();

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getRawY, m_rightStick::getRawTwist, m_rightStick::getRawThrottle));

        initAutos();
        configureButtonBindings();
        dashboardInit();
        if (Robot.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true); // silence joystick warnings in sim
    }   

    private void configureButtonBindings() {
        m_rightStick.getButton(1).whenPressed(m_drive::resetGyro);
        m_rightStick.getButton(2).whenPressed(m_drive::resetPose, m_drive);
    }

    private void initAutos() {

        try {
            for (int i = 0; i < trajectoryJson.length; i++) {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson[i]);
                trajectories[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            }
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJson, ex.getStackTrace());
        }
        
        auto = new SequentialCommandGroup(
            new RamseteCommand(trajectories[0], 
                                m_drive::getPose,
                                new RamseteController(Constants.DriveConstants.RAMSETE_B, Constants.DriveConstants.RAMSETE_ZETA),
                                new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                                                            Constants.DriveConstants.kV,
                                                            Constants.DriveConstants.kA),
                                Constants.DriveConstants.DRIVE_KINEMATICS,
                                m_drive::getWheelSpeeds,
                                new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                                new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                                m_drive::tankDriveVolts,
                                m_drive),
            new RamseteCommand(trajectories[1], 
                                m_drive::getPose,
                                new RamseteController(Constants.DriveConstants.RAMSETE_B, Constants.DriveConstants.RAMSETE_ZETA),
                                new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                                                            Constants.DriveConstants.kV,
                                                            Constants.DriveConstants.kA),
                                Constants.DriveConstants.DRIVE_KINEMATICS,
                                m_drive::getWheelSpeeds,
                                new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                                new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                                m_drive::tankDriveVolts,
                                m_drive),
            new InstantCommand(() -> m_drive.stop())
        );

        // auto = new RamseteCommand(Trajectories.trajectorySimple, 
        //                             m_drive::getPose,
        //                             new RamseteController(Constants.DriveConstants.RAMSETE_B, Constants.DriveConstants.RAMSETE_ZETA),
        //                             Constants.DriveConstants.DRIVE_KINEMATICS,
        //                             m_drive::setVelocityMpS,
        //                             m_drive)
        //                             .andThen(() -> m_drive.stop(), m_drive);
    }

    private void dashboardInit() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", m_commandScheduler);
            SmartDashboard.putData("Drivetrain", m_drive);
        }
            
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        m_drive.resetPose(trajectories[0].getInitialPose()); // change this if the trajectory being run changes
        return auto;
    }
}
