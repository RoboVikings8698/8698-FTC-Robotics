package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.MotionControl.Point;

import java.util.ArrayList;

//this class will hold paths specified by the developer, later, they will be interpolated and motion profile applied
public class DrivePaths {


    ArrayList<Point> path1;
    ArrayList<Point> path2;
    ArrayList<Point> path3;

    // Constructor
    public DrivePaths() {
        //path 1 HERE
        path1 = new ArrayList<>();
        path1.add(new Point(0, 0, 0));
        path1.add(new Point(2, 10, 0));
        path1.add(new Point(4, 3, 0));
        path1.add(new Point(10, 15, 0));
        path1.add(new Point(11, 4, 0));
        path1.add(new Point(11.1, 4.1, 0));

        //Path 2 HERE
        path2 = new ArrayList<>();
        path2.add(new Point(0, 0, 0));
        path2.add(new Point(2, 4, 0));
        path2.add(new Point(6, 12, 0));
        path2.add(new Point(8, 10, 0));
        path2.add(new Point(10, 6, 0));
        path2.add(new Point(12, 4, 0));

        //Path 3 HERE
        path3 = new ArrayList<>();
        path3.add(new Point(0, 0, 0));
        path3.add(new Point(2, 4, 0));
        path3.add(new Point(10, 5, 0));
        path3.add(new Point(15, 8, 0));
        path3.add(new Point(18, 20, 0));
        path3.add(new Point(20, 4, 0));

    }

    // Optionally, add methods to retrieve or manipulate paths
    public ArrayList<Point> getPath1() {
        return path1;
    }
    public ArrayList<Point> getPath2() {
        return path2;
    }
    public ArrayList<Point> getPath3() {
        return path3;
    }




}
