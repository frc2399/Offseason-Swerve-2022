package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class assigns specific movement and turning values to the robot
 * Used primarily for debugging and autonomous
 */

public class InputCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final double xSpeed, ySpeed, turningSpeed;
    private final boolean fieldOriented;
    //private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /**
     * Creates a new InputCmd
     * @param swerveSubsystem
     * @param xSpeed if field oriented, then x direction relative to field. Else, x direction relative to robot - Maisie (Herb)
     * @param ySpeed if field oriented, then y direction relative to field. Else, y direction relative to robot - Maisie (Herb)
     * @param turningSpeed speed of turning (positive CW) - Herb
     * @param fieldOriented if true then operate in field oriented mode - Herb
     */
    public InputCmd(SwerveSubsystem swerveSubsystem,
           double xSpeed, double ySpeed, double turningSpeed,
            boolean fieldOriented) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.turningSpeed = turningSpeed;
        this.fieldOriented = fieldOriented;
        // this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        // this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        // this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // 1. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
