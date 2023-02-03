package com.fairportrobotics.frc.poe.DriveSystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private SwerveModuleConfig config;

    private WPI_TalonFX driveFalcon;
    private WPI_TalonFX swerveFalcon;
    private CANCoder encoder;

    private double offset;

    private PIDController driveController;
    private ProfiledPIDController swerveController;

    private SimpleMotorFeedforward driveFeedForward;
    private SimpleMotorFeedforward swerveFeedForward;

    private String name;
    private long previousTime;

    public SwerveModule(SwerveModuleConfig config, int driveFalconID, int swerveFalconID, int encoderID, String name, double offset) {
        this.config = config;
        driveFalcon = new WPI_TalonFX(driveFalconID);
        swerveFalcon = new WPI_TalonFX(swerveFalconID);
        encoder = new CANCoder(encoderID);

        this.offset = offset;
        this.name = name;

        driveController = new PIDController(.3, 0, 0);
        swerveController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(config.getMaxAngleVel() , config.getMaxAngleAcc())
        );
        swerveController.enableContinuousInput(0, 360);
        encoder.setPositionToAbsolute();
        swerveFalcon.setInverted(true);
        driveFeedForward = new SimpleMotorFeedforward(0, 0);
        swerveFeedForward = new SimpleMotorFeedforward(0, 0);
    }

    public double getAngle() {
        return encoder.getPosition() - offset;
    }

    public SwerveModulePosition getPosition() {
        double distance = driveFalcon.getSelectedSensorPosition() / config.getEncoderTicksPerMeter();
        SmartDashboard.putNumber(name + "Distance Meters", distance);
        return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getAngle()));
    }

    public double getVelocity() {
        return driveFalcon.getSelectedSensorVelocity() * 10 / config.getEncoderTicksPerMeter();
    }

    public void fromModuleState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
        double driveFalconVoltage = driveController.calculate(getVelocity(),state.speedMetersPerSecond);
        double swerveFalconVoltage = swerveController.calculate(getAngle(), state.angle.getDegrees()) + swerveFeedForward.calculate(swerveController.getSetpoint().velocity);
        
        driveFalcon.set(ControlMode.PercentOutput,driveFalconVoltage);
        swerveFalcon.setVoltage(swerveFalconVoltage);

        
        // SmartDashboard.putNumber(name + " Swerve Voltage", swerveFalconVoltage);
         SmartDashboard.putNumber(name + " Drive Voltage", driveFalconVoltage);
        // SmartDashboard.putNumber(name + " Current Angle", getAngle() );
         SmartDashboard.putNumber(name + " Current Velocity", getVelocity() );
        // SmartDashboard.putNumber(name + " Target Angle", state.angle.getDegrees());
         SmartDashboard.putNumber(name + " Target Velocity", state.speedMetersPerSecond);
        // SmartDashboard.putNumber(name + " Swerve Target Velocity", (swerveController.getSetpoint().velocity));
    }

}