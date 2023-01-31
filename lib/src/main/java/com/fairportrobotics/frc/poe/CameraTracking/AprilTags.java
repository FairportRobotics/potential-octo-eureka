package com.fairportrobotics.frc.poe.CameraTracking;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class AprilTags { 
    // an ArrayList to store the cameras
    public final ArrayList<PhotonCamera> cameras = new ArrayList<>();

    public AprilTags(String...cameraNames) {
        for(String cameraName : cameraNames) {
            cameras.add(new PhotonCamera(cameraName));
        }
    }

    // Method to get a list of all targets from all cameras
    public List<PhotonTrackedTarget> getTargets() {
        List<PhotonTrackedTarget> targets = new ArrayList<>();

        for(PhotonCamera camera : cameras) {
            for(PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
                boolean toAdd = true;
                List<PhotonTrackedTarget> tempTargets = targets;
                // check if the target is already in the list and if it has a higher pose ambiguity
                for(PhotonTrackedTarget arTarget : targets) {
                    if(target.getFiducialId() == arTarget.getFiducialId() && target.getPoseAmbiguity() < arTarget.getPoseAmbiguity()) {
                        // if it does, remove it from the list
                        tempTargets.remove(arTarget);
                        break;
                    } else if(target.getFiducialId() == arTarget.getFiducialId()) {
                        // if the target is already in the list and has the same ID, don't add it
                        toAdd = false;
                        break;
                    }
                }
                if(toAdd) { tempTargets.add(target); }

                targets = tempTargets;
            }
        }

        return targets;
    }

    // method to check if any camera has any targets
    public boolean hasTargets() {
        return cameras.stream().anyMatch((camera) -> camera.getLatestResult().hasTargets() == true);
    }      

    // method to get the closest target
    public PhotonTrackedTarget getClosestTarget() {
        if(!hasTargets()) { return null; }
        PhotonTrackedTarget closestTarget = null;
        for(PhotonTrackedTarget target : getTargets()) {
            if(closestTarget == null || calculateDistance(closestTarget.getBestCameraToTarget()) > calculateDistance(target.getBestCameraToTarget())) { closestTarget = target; }
        }
        return closestTarget;
    }

    // method to get the target with the lowest pose ambiguity
    public PhotonTrackedTarget getLowestAmbiguityTarget() {
        if(!hasTargets()) { return null; }
        PhotonTrackedTarget lowestAmbiguity = null;
        for(PhotonTrackedTarget target : getTargets()) {
            // check if the target has a lower pose ambiguity than the current lowest ambiguity target
            if(lowestAmbiguity == null || lowestAmbiguity.getPoseAmbiguity() > target.getPoseAmbiguity()) { lowestAmbiguity = target; }
        }
        return lowestAmbiguity;
    }

    private double calculateDistance(Transform3d t) {
        return Math.sqrt(t.getX()*t.getX() + t.getY()*t.getY());
    }
}