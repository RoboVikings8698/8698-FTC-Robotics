package org.firstinspires.ftc.teamcode.subsystems.MotionControl;


import androidx.annotation.NonNull;

//this class creates object of an point, saving x position, y position, and theta bearing angle
public class Point {

    private double x; //stores x point coordinate
    private double y; //stores y point coordinate
    private double theta; //stores bearing angle

    //constructor
    public Point(double x, double y, double theta) {

        this.x = x;
        this.y = y;
        this.theta = theta;

    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public double getTheta() {
        return theta;
    }


    //override if you want to print this stuff as strign
    // Optional: toString() method to display point values easily
    @NonNull
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + theta + ")";
    }


}
