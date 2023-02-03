package com.fairportrobotics.frc.poe.DriveSystems.SwerveDrive;

public class SwerveModuleConfig {
    
    private double maxAngleAcc = 0.0;
    private double maxAngleVel = 0.0;
    private double encoderTicksPerMeter = 0.0;

    public SwerveModuleConfig(double maxAngleAcc, double maxAngleVel, double encoderTicksPerMeter){
        this.maxAngleAcc = maxAngleAcc;
        this.maxAngleVel = maxAngleVel;
        this.encoderTicksPerMeter = encoderTicksPerMeter;
    }

    public double getMaxAngleVel(){
        return maxAngleVel;
    }

    public double getMaxAngleAcc(){
        return maxAngleAcc;
    }

    public double getEncoderTicksPerMeter(){
        return encoderTicksPerMeter;
    }

}
