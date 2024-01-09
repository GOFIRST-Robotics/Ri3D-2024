// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera(Constants.USB_CAMERA_NAME); // Declare the name of the camera used in the pipeline
    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result; // Stores all the data that Photonvision returns

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult(); // Query the latest result from PhotonVision
        hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
        if (hasTarget) {
            this.result = result;
        }
        boolean in_range = InRange(0, 5, 0, 5);
    }
    public PhotonTrackedTarget getTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it exists)
        List<PhotonTrackedTarget> targets = result.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }
    
    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
        return result.getBestTarget(); // Returns the best (closest) target
        }
        else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }
    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }

    public double getDistanceToTarget(PhotonTrackedTarget target) {
        if (!hasTarget) {
            return 0;
        }
        double april_tag_pitch = target.getPitch();
        double april_tag_area = target.getArea();

        double distance = april_tag_area;

        // Print the area and pitch of the target
        //System.out.println("Area: " + april_tag_height + "Pitch: " + april_tag_pitch);
        SmartDashboard.putNumber("t_area", april_tag_area);
        SmartDashboard.putNumber("t_pitch", april_tag_pitch);
        return distance;
    }

    public boolean InRange(double distanceThreshold, double distanceThresholdRange,
    double angleThreshold, double angleThresholdRange) {
        if (!hasTarget) {
            return false;
        }
    
        PhotonTrackedTarget bestTarget = getBestTarget();
        double distanceToTarget  = getDistanceToTarget(bestTarget);
        double angleToTarget = bestTarget.getYaw(); // Assuming yaw gives the angle
        double skewTarget = bestTarget.getSkew();

        //boolean inRange = Math.abs(distanceToTarget) <= distanceThreshold && Math.abs(angleToTarget) <= angleThreshold;
        boolean inRange = Math.abs(Math.abs(distanceToTarget) - distanceThreshold) >= distanceThresholdRange && Math.abs(Math.abs(angleToTarget) - angleThreshold) >= angleThresholdRange;
        SmartDashboard.putNumber("t_distance", distanceToTarget);
        
        SmartDashboard.putNumber("t_angle", angleToTarget);

        SmartDashboard.putNumber("t_skew", skewTarget);

        SmartDashboard.putBoolean("InRange", inRange);
    
        return inRange;
    }
}

// I need to modify 'periodic()' to call a new function 'InRange()' that returns a boolean value if the target is within a distance and angle range 