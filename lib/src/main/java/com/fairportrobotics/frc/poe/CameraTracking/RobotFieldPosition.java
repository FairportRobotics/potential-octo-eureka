package com.fairportrobotics.frc.poe.CameraTracking;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class RobotFieldPosition {   
    AprilTagFieldLayout layout;
    PhotonPoseEstimator photonPoseEstimator;

    /*
        Example of Transform3d: 

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 
        Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    */
    public RobotFieldPosition(String camera, Transform3d robotToCam, String layoutResourceFile) throws IOException {

        layout = AprilTagFieldLayout.loadFromResource(layoutResourceFile);
        
        PhotonCamera cam = new PhotonCamera(camera);
        photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, cam, robotToCam);
    }  

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }


}   