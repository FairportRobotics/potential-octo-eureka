package org.fairportrobotics.frc.poe.DriveSystems.CTRESwerveDrive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 
 */
public class CTRESwerveDriveSubsystem extends SubsystemBase{
    
    private SwerveDrivetrain _SwerveDrivetrain;

    public CTRESwerveDriveSubsystem(){
        
        SwerveModuleConstantsFactory constantsFactory = new SwerveModuleConstantsFactory()
            .withWheelRadius(1.975);

        SwerveModuleConstants flConstants = constantsFactory.createModuleConstants(0, 0, 0, 0, 0, 0, false);
        SwerveModuleConstants frConstants = constantsFactory.createModuleConstants(0, 0, 0, 0, 0, 0, false);
        SwerveModuleConstants brConstants = constantsFactory.createModuleConstants(0, 0, 0, 0, 0, 0, false);
        SwerveModuleConstants blConstants = constantsFactory.createModuleConstants(0, 0, 0, 0, 0, 0, false);

        _SwerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainConstants(), flConstants, frConstants, brConstants, blConstants);
    }

    /**
     * Drive the robot in a Field Centric maner.
     * @param vX Velocity in the X direction, in m/s.
     * @param vY Velocity in the Y direction, in m/s.
     * @param vOmega Angular rate to rotate at, in radians/s.
     */
    public void driveFieldCentric(double vX, double vY, double vOmega){
        FieldCentric request = new FieldCentric()
            .withVelocityX(vX)
            .withVelocityY(vY)
            .withRotationalRate(vOmega);
        _SwerveDrivetrain.setControl(request);
    }

    /**
     * Drive the robot in a Robot Centric maner.
     * @param vX Velocity in the X direction, in m/s.
     * @param vY Velociry in the Y direction, in m/s.
     * @param vOmega Angular rate to rotate at, in radians/s.
     */
    public void driveRobotCentric(double vX, double vY, double vOmega){
        RobotCentric request = new RobotCentric()
            .withVelocityX(vX)
            .withVelocityY(vY)
            .withRotationalRate(vOmega);
        _SwerveDrivetrain.setControl(request);
    }

}
