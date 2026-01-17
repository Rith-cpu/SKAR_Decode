package org.firstinspires.ftc.teamcode.auto;

public class Trig {
    public static double arcsin(double distance, double deltaY) {
        return Math.asin(deltaY / distance);
    }
    public double distance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2-x1,2)+Math.pow(y2-y1,2));
    }


}
