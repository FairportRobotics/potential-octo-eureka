package com.fairportrobotics.frc.poe.Sensors.Gyro;

import com.kauailabs.navx.frc.AHRS;

public class GyroSubsystem {
    private AHRS gyro;

    public GyroSubsystem() {
        gyro = new AHRS();
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    public void reset() {
        gyro.reset();
    }
}
