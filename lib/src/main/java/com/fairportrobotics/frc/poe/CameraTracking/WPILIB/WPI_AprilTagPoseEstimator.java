package com.fairportrobotics.frc.poe.CameraTracking.WPILIB;

import java.io.IOException;
import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionRunner;

public class WPI_AprilTagPoseEstimator {

    private final AprilTagDetector aprilTagDetector;
    private final AprilTagPoseEstimator aprilTagPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Pose3d estimatedPose = new Pose3d();

    private VisionRunner<AprilTagsDetectorPipeline> visionRunner;

    public WPI_AprilTagPoseEstimator(VideoCamera imageSource) {
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.addFamily("tag16h5");
        AprilTagPoseEstimator.Config poseEstimatorConfig = new Config(0.203, 0, 0, 0, 0);
        aprilTagPoseEstimator = new AprilTagPoseEstimator(poseEstimatorConfig);

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        visionRunner = new VisionRunner<AprilTagsDetectorPipeline>(imageSource, new AprilTagsDetectorPipeline(), new VisionSystemCallback());
    }

    public void runDetectorPipeline(){
        new Thread(() -> {
            try{
                visionRunner.runForever();
            } catch (Exception e){
                e.printStackTrace();
            }
        }).start();
    }

    public Pose3d getEstimatedGlobalPose() {
        return estimatedPose;
    }

    private final class AprilTagsDetectorPipeline implements VisionPipeline {

        private Pose3d estPose = new Pose3d();

        @Override
        public void process(Mat image) {
            AprilTagDetection[] aprilTags = aprilTagDetector.detect(image);

            for (AprilTagDetection aprilTag : aprilTags) {
                Transform3d tagRelativeToRobot = aprilTagPoseEstimator.estimate(aprilTag);
                Optional<Pose3d> tagRelativeToFieldOptional = aprilTagFieldLayout.getTagPose(aprilTag.getId());

                if (tagRelativeToFieldOptional.isPresent()) {
                    Pose3d tagRelativeToFieldPose = tagRelativeToFieldOptional.get();
                    Pose3d estimatedFieldPos = tagRelativeToFieldPose.plus(tagRelativeToRobot);
                    // Logger.getInstance().recordOutput("April Tag Pose", tagRelativeToFieldPose);
                    estPose = estimatedFieldPos;
                }
            }
        }

        public Pose3d getEstimatedPose(){
            return estPose;
        }

    }

    private final class VisionSystemCallback implements VisionRunner.Listener<AprilTagsDetectorPipeline> {
        @Override
        public void copyPipelineOutputs(AprilTagsDetectorPipeline pipeline) {
            estimatedPose = pipeline.getEstimatedPose();
        }
    }

}
