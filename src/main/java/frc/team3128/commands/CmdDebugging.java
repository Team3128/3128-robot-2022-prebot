package frc.team3128.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdDebugging extends CommandBase {
    private final Limelight ballLimelight;

    public CmdDebugging(Limelight ballLimelight) {
        this.ballLimelight = ballLimelight;
    }

    @Override
    public void initialize() {
        ballLimelight.setLEDMode(LEDMode.ON);
    }

    @Override
    public void execute() {
        double previousVerticalAngle = ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2);
        double approxDistance = ballLimelight.calculateDistToGroundTarget(previousVerticalAngle, Constants.VisionContants.BALL_TARGET_HEIGHT / 2);
        SmartDashboard.putNumber("Distance", approxDistance);
        SmartDashboard.putNumber("Vertical Angle", previousVerticalAngle);
        Log.info("Debugging", "Command running");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ballLimelight.setLEDMode(LEDMode.OFF);
    }

}