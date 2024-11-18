package org.firstinspires.ftc.teamcode.subsystems.MotionControl;

import java.util.ArrayList;

public class MotionProfile {

    private double maxVelocity;
    private double maxAcceleration;
    private double timeStep; // Time step for each profile point (e.g., 0.02 seconds)

    // Constructor to initialize the constraints
    public MotionProfile(double maxVelocity, double maxAcceleration, double timeStep) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.timeStep = timeStep;
    }

    // Generate a motion profile from a list of interpolated points
    public ArrayList<MotionState> generateProfile(ArrayList<Point> points) {
        ArrayList<MotionState> profile = new ArrayList<>();

        double previousX = points.get(0).getX();
        double previousY = points.get(0).getY();
        double previousVelocity = 0;
        double currentVelocity = 0;
        double distance = 0;

        for (int i = 1; i < points.size(); i++) {
            Point p1 = points.get(i - 1);
            Point p2 = points.get(i);

            // Calculate the distance between points
            distance = Math.sqrt(Math.pow(p2.getX() - p1.getX(), 2) + Math.pow(p2.getY() - p1.getY(), 2));

            // Time to cover the distance, assuming max velocity
            double timeToMaxVelocity = (maxVelocity - previousVelocity) / maxAcceleration;
            double distanceToMaxVelocity = (previousVelocity * timeToMaxVelocity) +
                    (0.5 * maxAcceleration * Math.pow(timeToMaxVelocity, 2));

            // Check if max velocity is reached, or if we need to decelerate before max velocity
            if (distance > 2 * distanceToMaxVelocity) {
                // Accelerate to max velocity, hold max velocity, then decelerate
                currentVelocity = maxVelocity;
            } else {
                // Only accelerate and decelerate within the distance
                currentVelocity = Math.sqrt(previousVelocity * previousVelocity + 2 * maxAcceleration * distance);
            }

            // Calculate time to travel the distance at current velocity
            double travelTime = distance / currentVelocity;

            // Add the motion state for this segment
            profile.add(new MotionState(p1.getX(), p1.getY(), currentVelocity, maxAcceleration, travelTime));

            // Update the previous values for the next iteration
            previousVelocity = currentVelocity;
            previousX = p2.getX();
            previousY = p2.getY();
        }

        return profile;
    }
}