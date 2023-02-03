package com.fairportrobotics.frc.poe.drivesystems.swervedrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

<<<<<<< Updated upstream
public class SwerveDriveSubsystem extends SubsystemBase{
    
    private WPI_TalonFX frontLeftDrive;

    public SwerveDriveSubsystem(){
        
=======

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveModule leftFrontModule;
    private SwerveModule rightFrontModule;
    private SwerveModule leftBackModule;
    private SwerveModule rightBackModule;

    private SwerveDriveKinematics kinematics;
    private GyroSubsystem gyro;

    private SwerveDriveOdometry odometry;

    public DriveSubsystem(GyroSubsystem gyro) {
        this.gyro = gyro;

        leftFrontModule = new SwerveModule(Constants.FRONT_LEFT_DRIVE_ID, Constants.FRONT_LEFT_SWERVE_ID, Constants.FRONT_LEFT_ENCODER_ID, "Front Left", Constants.FRONT_LEFT_SWERVE_OFFSET);
        leftBackModule = new SwerveModule(Constants.BACK_LEFT_DRIVE_ID, Constants.BACK_LEFT_SWERVE_ID, Constants.BACK_LEFT_ENCODER_ID, "Front Right", Constants.FRONT_RIGHT_SWERVE_OFFSET);
        rightFrontModule = new SwerveModule(Constants.FRONT_RIGHT_DRIVE_ID, Constants.FRONT_RIGHT_SWERVE_ID, Constants.FRONT_RIGHT_ENCODER_ID, "Back Left", Constants.BACK_LEFT_SWERVE_OFFSET);
        rightBackModule = new SwerveModule(Constants.BACK_RIGHT_DRIVE_ID, Constants.BACK_RIGHT_SWERVE_ID, Constants.BACK_RIGHT_ENCODER_ID, "Back Right", Constants.BACK_RIGHT_SWERVE_OFFSET);

        Translation2d frontLeftLocation = new Translation2d(11.25, 10);
        Translation2d frontRightLocation = new Translation2d(-11.25, 10);
        Translation2d backLeftLocation = new Translation2d(11.25, -10);
        Translation2d backRightLocation = new Translation2d(-11.25, -10);

        kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        );


        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[]{
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftBackModule.getPosition(),
            rightBackModule.getPosition()
        });

       
>>>>>>> Stashed changes
    }

    public void drive(double forward, double strafe, double rotate, double yaw) {
        SmartDashboard.putNumber("yaw", yaw);
        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotate/10, Rotation2d.fromDegrees(-yaw));
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(velocity);
        leftFrontModule.fromModuleState(moduleStates[0]);
        leftBackModule.fromModuleState(moduleStates[1]);
        rightFrontModule.fromModuleState(moduleStates[2]);
        rightBackModule.fromModuleState(moduleStates[3]);
    }

     @Override
     public void periodic() {
        odometry.update(Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[]{
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftBackModule.getPosition(),
            rightBackModule.getPosition()
        });
        SmartDashboard.putNumber("Odometry X Meters", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y Meters", odometry.getPoseMeters().getY());
     }

     public Pose2d getPosition() {
         return odometry.getPoseMeters();
     }

     public double getAverageVelocity() {
         return (Math.abs(leftFrontModule.getVelocity()) +
         Math.abs(leftFrontModule.getVelocity()) + 
         Math.abs(leftFrontModule.getVelocity()) +
         Math.abs(leftFrontModule.getVelocity()))/4;
     }
}