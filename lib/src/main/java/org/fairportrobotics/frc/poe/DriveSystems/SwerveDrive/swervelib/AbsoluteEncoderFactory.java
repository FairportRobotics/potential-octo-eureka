package org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive.swervelib;

@FunctionalInterface
public interface AbsoluteEncoderFactory<Configuration> {
    AbsoluteEncoder create(Configuration configuration);
}
