package com.fairportrobotics.frc.poe.drivesystems.tankdrive.command;

import com.fairportrobotics.frc.poe.drivesystems.tankdrive.subsystem.TankDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {

    private TankDriveSubsystem driveSubsystem;

	public TankDriveCommand() {
		
	}

	@Override
    public void initialize() {
		// TODO Auto-generated method stub
	}

	@Override
    public void execute() {
		// TODO Auto-generated method stub

		double leftJoystickValue = OI.getLeft();
		double rightJoystickValue = OI.getRight();

		driveSubsystem.drive(leftJoystickValue, rightJoystickValue);

	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}