package org.firstinspires.ftc.teamcode.subsystems;

import java.lang.Math;

public class Functions {


    public static double DeadZone(double input, double deadZone) {
        if(Math.abs(input) < deadZone)
        {
            return 0;
        }
        else
        {
            return input;
        }
    }
    public static double Exponential(double input) {
        return input * Math.abs(input);
    }
    public static double TriangleWave(double x) {
        return 2*(Math.asin(Math.sin(2*Math.PI*x)))/Math.PI;
    }
    public static double Pythagorean(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    }

    //calculate vector magnitude
    public static double VectorMagnitude(double x, double y){
        return Math.abs(Math.sqrt((x * x) + (y * y)));
    }

    //delta angle
    public static double DeltaAngleDeg(double startAngle, double endAngle) {
        return((((endAngle - startAngle - 180) % 360) +360) % 360) - 180;
    }

    public static double DeltaAngleRadians(double startAngle, double endAngle) {
        return ((((endAngle - startAngle - Math.PI) % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI))-Math.PI;
    }

    //useful function to clamp things down
    public static double Clamp (double in, double min, double max) {
        return Math.min(Math.max(in,min),max);
    }

    //same thing but clams to minimum
    public static double ClampMin (double in, double min) {
        return Math.max(in,min);
    }

    //same thing but clamps to maximum
    public static double ClampMax (double in, double max) {
        return Math.min(in,max);
    }

    //you have array, get largest value out of it
    public static double Max (double[]values) {
        double max = values[0];
        for (int i = 0; i < values.length; i++) {
            max = (values[i]>max)?values[i]:max;
        }
        return max;
    }

    public static boolean InRangeOf(double number, double referenceNumber, double rangeOf) { return (referenceNumber + rangeOf > number && referenceNumber - rangeOf < number); }
    // graph link https://www.desmos.com/calculator/ccaguak1ra
    public static double AltAxisCoord(double x, double y, double a) //two graph planes, on over another, you have point on the bottom one, and you have know angle offset of the top one relative tot eh bottom one, function will return coordinates of the top one
    {
        double s = Math.sin(a)*Math.cos(a)*(x*Cot(a)+y);
        return (Math.tan(a)*s<0?(Math.sin(a)>0?-1:1):(Math.sin(a)<0?-1:1))
                * Math.sqrt((s*s)+((Math.tan(a)*(s))*(Math.tan(a)*(s))));
    }
    public static double Cot(double a)
    {
        return Math.cos(a) / Math.sin(a);
    }
    public static String ContainsMultiple(String s) {
        for (int i = 0; i < s.length(); i++) {
            if (s.substring(1).indexOf(s.substring(0,1)) != -1) return s.substring(0,1);
            s = s.substring(1);
        }
        return "";
    }
    public static boolean ContainsMultipleOf(String s, String in) {
        s = s.substring(s.indexOf(in));
        return s.substring(in.length()).indexOf(in) != -1;
    }
    public static String RemoveMultiples(String s) {
        while (!ContainsMultiple(s).equals("")) {
            String sub = ContainsMultiple(s);
            int i = s.substring(s.indexOf(sub) + 1).indexOf(sub);
            s = s.substring(0,i) + s.substring(i+1);
        }
        return s;
    }
}
