package org.firstinspires.ftc.teamcode.subsystems.MotionControl;

import java.util.ArrayList;

public class CubicSplineInterpolator {

    private double[] a;  // Coefficients for f(x)
    private double[] b;  // Coefficients for f'(x)
    private double[] c;  // Coefficients for f''(x)
    private double[] d;  // Coefficients for f'''(x)
    private double[] x;  // The x-coordinates of control points
    private int n;       // Number of splines



    // Constructor to initialize the control points
    public CubicSplineInterpolator(ArrayList<Point> points) {

        n = points.size() - 1;  // Number of intervals

        // Extract x and y coordinates from the points
        x = new double[points.size()];
        double[] y = new double[points.size()];

        for (int i = 0; i < points.size(); i++) {
            x[i] = points.get(i).getX();
            y[i] = points.get(i).getY();
        }

        // Initialize coefficient arrays
        a = new double[n + 1];
        b = new double[n];
        c = new double[n + 1];
        d = new double[n];

        // Natural cubic spline computation (zero second derivatives at the endpoints)
        computeCoefficients(x, y);
    }

    private void computeCoefficients(double[] x, double[] y) {
        double[] h = new double[n];
        double[] alpha = new double[n];
        double[] l = new double[n + 1];
        double[] mu = new double[n];
        double[] z = new double[n + 1];

        // Compute the intervals h[i]
        for (int i = 0; i < n; i++) {
            h[i] = x[i + 1] - x[i];
        }

        // Compute alpha values (skip alpha[0] because it's not needed in a natural spline)
        for (int i = 1; i < n; i++) {
            alpha[i] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
        }

        // Step 2: Decomposition step
        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;

        // Forward sweep
        for (int i = 1; i < n; i++) {
            l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        // Set the last l[n], z[n], and c[n] for natural cubic spline
        l[n] = 1;
        z[n] = 0;
        c[n] = 0;

        // Backward substitution step
        for (int j = n - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3;
            d[j] = (c[j + 1] - c[j]) / (3 * h[j]);
            a[j] = y[j];
        }
    }


    // Generate interpolated points
    public ArrayList<Point> interpolate(double resolution) {
        ArrayList<Point> interpolatedPoints = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            for (double t = x[i]; t <= x[i + 1]; t += resolution) {
                double delta = t - x[i];
                double interpolatedX = t;
                double interpolatedY = a[i] + b[i] * delta + c[i] * delta * delta + d[i] * delta * delta * delta;

                // For this example, we'll just assume the bearing angle (theta) stays constant
                interpolatedPoints.add(new Point(interpolatedX, interpolatedY, 0));  // Assuming a constant theta
            }
        }

        return interpolatedPoints;
    }
}