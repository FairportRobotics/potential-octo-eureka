package com.fairportrobotics.frc.poe.drivesystems.tankdrive.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase{
    
	// variables for talons
	private WPI_TalonSRX rightMaster;
	private WPI_TalonSRX rightSlave;
	private WPI_TalonSRX leftMaster;
	private WPI_TalonSRX leftSlave;

	public TankDriveSubsystem(int leftMasterId, int leftSlaveId, int rightMasterId, int rightSlaveId){
		rightMaster = new WPI_TalonSRX(rightMasterId);
		rightSlave = new WPI_TalonSRX(rightSlaveId);
		leftMaster = new WPI_TalonSRX(leftMasterId);
		leftSlave = new WPI_TalonSRX(leftSlaveId);
	}

	// assigns the talon variables to an ID from RobotMap.java
	public void initialize() {
		rightMaster.setSafetyEnabled(true);
		rightSlave.setSafetyEnabled(false);
		leftMaster.setSafetyEnabled(true);
		leftSlave.setSafetyEnabled(false);

		rightMaster.setExpiration(.25);
		leftMaster.setExpiration(.25);

		rightSlave.follow(rightMaster);
		leftSlave.follow(leftMaster);

		rightMaster.set(0);
		leftMaster.set(0);
	}

	public void drive(double left, double right) {

		leftMaster.set(left);
		rightMaster.set(-right);

	}

}
