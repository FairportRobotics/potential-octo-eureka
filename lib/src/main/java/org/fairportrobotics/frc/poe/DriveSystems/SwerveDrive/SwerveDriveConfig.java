package org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive;

import org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive.swervelib.MechanicalConfiguration;
import org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive.swervelib.MkSwerveModuleBuilder;
import org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive.swervelib.MotorType;
import org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive.swervelib.SwerveModule;

public class SwerveDriveConfig {

    private static final MechanicalConfiguration MK4I_L1 = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            false,
            (14.0 / 50.0) * (10.0 / 60.0),
            false);

    private double drivetrainWheelbase;
    private double drivetrainTrackwidth;

    private int gyroCanId;

    private SwerveModule frontLeftSwerveModule;
    private SwerveModule frontRightSwerveModule;
    private SwerveModule backRightSwerveModule;
    private SwerveModule backLeftSwerveModule;

    public double getDrivetrainWheelbase() {
        return drivetrainWheelbase;
    }

    public double getDrivetrainTrackwidth() {
        return drivetrainTrackwidth;
    }

    public int getGyroCanId() {
        return gyroCanId;
    }

    public SwerveModule getFLSwerveModule() {
        return frontLeftSwerveModule;
    }

    public SwerveModule getFRSwerveModule() {
        return frontRightSwerveModule;
    }

    public SwerveModule getBRSwerveModule() {
        return backRightSwerveModule;
    }

    public SwerveModule getBLSwerveModule() {
        return backLeftSwerveModule;
    }

    private SwerveDriveConfig(SwerveDriveConfigBuilder builder) {
        this.drivetrainTrackwidth = builder.drivetrainTrackwidth;
        this.drivetrainWheelbase = builder.drivetrainWheelbase;
        this.gyroCanId = builder.pigeonIMUCanId;

        this.frontLeftSwerveModule = builder.frontLeftModule;
        this.frontRightSwerveModule = builder.frontRightModule;
        this.backRightSwerveModule = builder.backRightModule;
        this.backLeftSwerveModule = builder.backLeftModule;
    }

    public static class SwerveDriveConfigBuilder {
        private double drivetrainWheelbase;

        public SwerveDriveConfigBuilder setWheelbase(double wheelbaseInMeters) {
            drivetrainWheelbase = wheelbaseInMeters;
            return this;
        }

        private double drivetrainTrackwidth;

        public SwerveDriveConfigBuilder setTrackwidth(double trackwidthInMeters) {
            drivetrainTrackwidth = trackwidthInMeters;
            return this;
        }

        private int pigeonIMUCanId;

        public SwerveDriveConfigBuilder setPigeonCANId(int pigeonCANId) {
            pigeonIMUCanId = pigeonCANId;
            return this;
        }

        private SwerveModule frontLeftModule;

        public SwerveDriveConfigBuilder setFLModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset) {
            frontLeftModule = new MkSwerveModuleBuilder()
                    .withGearRatio(MK4I_L1)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        public SwerveDriveConfigBuilder setFLModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset, MechanicalConfiguration mechanicalConfiguration) {
            frontLeftModule = new MkSwerveModuleBuilder()
                    .withGearRatio(mechanicalConfiguration)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        private SwerveModule frontRightModule;

        public SwerveDriveConfigBuilder setFRModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset) {
            frontRightModule = new MkSwerveModuleBuilder()
                    .withGearRatio(MK4I_L1)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        public SwerveDriveConfigBuilder setFRModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset, MechanicalConfiguration mechanicalConfiguration) {
            frontRightModule = new MkSwerveModuleBuilder()
                    .withGearRatio(mechanicalConfiguration)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        private SwerveModule backRightModule;

        public SwerveDriveConfigBuilder setBRModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset) {
            backRightModule = new MkSwerveModuleBuilder()
                    .withGearRatio(MK4I_L1)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        public SwerveDriveConfigBuilder setBRModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset, MechanicalConfiguration mechanicalConfiguration) {
            backRightModule = new MkSwerveModuleBuilder()
                    .withGearRatio(mechanicalConfiguration)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        private SwerveModule backLeftModule;

        public SwerveDriveConfigBuilder setBLModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset) {
            backLeftModule = new MkSwerveModuleBuilder()
                    .withGearRatio(MK4I_L1)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        public SwerveDriveConfigBuilder setBLModule(MotorType driveMotor, int driverMotorId, MotorType steerMotor,
                int steerMotorId, int steerEncoderId, double offset, MechanicalConfiguration mechanicalConfiguration) {
            backLeftModule = new MkSwerveModuleBuilder()
                    .withGearRatio(mechanicalConfiguration)
                    .withDriveMotor(driveMotor, driverMotorId)
                    .withSteerMotor(steerMotor, steerMotorId)
                    .withSteerEncoderPort(steerEncoderId)
                    .withSteerOffset(offset)
                    .build();
            return this;
        }

        public SwerveDriveConfig build() {
            return new SwerveDriveConfig(this);
        }
    }

}
