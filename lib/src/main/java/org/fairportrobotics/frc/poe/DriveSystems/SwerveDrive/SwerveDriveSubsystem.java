package org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive;

import org.fairportrobotics.frc.poe.CameraTracking.RobotFieldPosition;
import org.fairportrobotics.frc.poe.DriveSystems.SwerveDrive.swervelib.SwerveModule;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final double MAX_VOLTAGE = 12.0;
    private final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;

    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private SwerveModuleState[] moduleStates = { new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState() };

    private final Pigeon2 gyroscope;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private RobotFieldPosition fieldPositionEstimator;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SlewRateLimiter simVelocityX;
    private SlewRateLimiter simVelocityY;

    public SwerveDriveSubsystem(SwerveDriveConfig config) {

        swerveModules[0] = config.getFLSwerveModule();
        swerveModules[1] = config.getFRSwerveModule();
        swerveModules[2] = config.getBLSwerveModule();
        swerveModules[3] = config.getBRSwerveModule();

        kinematics = new SwerveDriveKinematics(
                new Translation2d(config.getDrivetrainTrackwidth() / 2.0,
                        config.getDrivetrainWheelbase() / 2.0),
                new Translation2d(config.getDrivetrainTrackwidth() / 2.0,
                        -config.getDrivetrainWheelbase() / 2.0),
                new Translation2d(-config.getDrivetrainTrackwidth() / 2.0,
                        config.getDrivetrainWheelbase() / 2.0),
                new Translation2d(-config.getDrivetrainTrackwidth() / 2.0,
                        -config.getDrivetrainWheelbase() / 2.0));

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation(), getModulePositions(),
                getPose());

        simVelocityX = new SlewRateLimiter(10);
        simVelocityY = new SlewRateLimiter(10);

        gyroscope = new Pigeon2(config.getGyroCanId());

    }

    public void zeroGyroscope() {
        poseEstimator.resetPosition(Rotation2d.fromDegrees(gyroscope.getAngle()),
                getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),
                        Rotation2d.fromDegrees(0.0)));
    }

    public void resetOdometry(Pose2d newPose2d) {
        poseEstimator.resetPosition(Rotation2d.fromDegrees(gyroscope.getAngle()),
                getModulePositions(),
                newPose2d);
    }

    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;

        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i < 4; i++) {
            SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromRadians(swerveModules[i].getSteerAngle()));
            swerveModules[i].set(
                    moduleStates[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND
                            * MAX_VOLTAGE,
                    moduleStates[i].angle.getRadians());
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pigeon2 getGyro() {
        return gyroscope;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(), swerveModules[3].getPosition() };
    }

    @Override
    public void periodic() {

        poseEstimator.update(Rotation2d.fromDegrees(gyroscope.getAngle()),
                getModulePositions());

        fieldPositionEstimator.getPhotonPoseEstimator().setReferencePose(poseEstimator.getEstimatedPosition());
    }

    @Override
    public void simulationPeriodic() {
        double x = getPose().getX();
        double y = getPose().getY();
        Rotation2d rotation = getRotation();

        double targetVelX = (chassisSpeeds.vxMetersPerSecond * Math.cos(rotation.getRadians())
                - chassisSpeeds.vyMetersPerSecond * Math.sin(rotation.getRadians()));
        double targetVelY = (chassisSpeeds.vxMetersPerSecond * Math.sin(rotation.getRadians())
                + chassisSpeeds.vyMetersPerSecond * Math.cos(rotation.getRadians()));
        Rotation2d dRotation = Rotation2d.fromRadians(-chassisSpeeds.omegaRadiansPerSecond * 0.02);

        x += simVelocityX.calculate(targetVelX) * 0.02;
        y += simVelocityY.calculate(targetVelY) * 0.02;
        rotation = rotation.rotateBy(dRotation);

        this.resetOdometry(new Pose2d(x, y, rotation));
    }
}
