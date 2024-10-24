package org.firstinspires.ftc.teamcode.subsystems.MotionControl;

import java.util.ArrayList;

public class LinearInterpolator {

    // Interpolates between a list of Point objects
    public ArrayList<Point> interpolate(ArrayList<Point> points, int numPointsBetween) {
        ArrayList<Point> interpolatedPoints = new ArrayList<>();

        // Loop through each pair of points
        for (int i = 0; i < points.size() - 1; i++) {
            Point p1 = points.get(i);
            Point p2 = points.get(i + 1);

            // Add the starting point of the segment
            interpolatedPoints.add(p1);

            // Linearly interpolate between p1 and p2
            for (int j = 1; j <= numPointsBetween; j++) {
                double t = (double) j / (numPointsBetween + 1);

                // Linearly interpolate x, y, and theta
                double interpolatedX = p1.getX() + t * (p2.getX() - p1.getX());
                double interpolatedY = p1.getY() + t * (p2.getY() - p1.getY());
                double interpolatedTheta = p1.getTheta() + t * (p2.getTheta() - p1.getTheta());

                // Create the interpolated point
                Point interpolatedPoint = new Point(interpolatedX, interpolatedY, interpolatedTheta);

                // Add the interpolated point to the list
                interpolatedPoints.add(interpolatedPoint);
            }
        }

        // Add the final point
        interpolatedPoints.add(points.get(points.size() - 1));

        return interpolatedPoints;
    }
}