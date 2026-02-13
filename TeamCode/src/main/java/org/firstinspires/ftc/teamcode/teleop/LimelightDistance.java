package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightDistance {

    private Limelight3A limelight;

    double usedDistance;
    double calcDistanceLow;
    double calcDistanceMid;
    double calcDistanceHigh;


    double m100 = 3.33499;
    double b100 = 1061.85831; // 1071.85831 new -10

    double EQresult100;

    double m50 = 4.6945;
    double b50 = 968.67251; // 991.50633, 980.50633

    double EQresult50;
    double m = 4.82433;
    double b = 973.16097; // 0.3669

    double EQresult;

    public double lowVelocity = 0.0;
    public double accVelocity = 0.0;
    String[] equation = { "Low", "Mid", "High"};

    int stepIndex = 0;


    public LimelightDistance(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public double estimateDistanceFromTa(double ta){
        double a = 3197.163;
        double b = -1.914009;
        return Math.pow(ta / a, 1.0 / b);
    }

    public Double getDistanceIfValid() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimateDistanceFromTa(r.getTa());
    }
    public double estimatePowerFromDistanceGreaterThan100(double distance){
        return calcDistanceHigh;
    }

    public Double getHighPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceGreaterThan100(distance);
    }

    public double estimatePowerFromDistanceLessThan100GreaterThan50(double distance){
        return calcDistanceMid;
    }

    public Double getLowPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceLessThan100GreaterThan50(distance);
    }

    public double estimatePowerFromDistanceLessThan50(double distance){
        return calcDistanceLow;
    }

    public Double getLowLowPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceLessThan50(distance);
    }

    public Double getDistance(double distance){
        usedDistance = distance;
        return usedDistance;
    }

    public void customFlyweelVel(Gamepad gamepad) {
        if (gamepad.dpadLeftWasPressed()){
            stepIndex = (stepIndex - 1) % equation.length;
        }
        if (gamepad.dpadRightWasPressed()){
            stepIndex = (stepIndex + 1) % equation.length;
        }
        if (equation[Math.abs(stepIndex)].equals("Low")){
             if (gamepad.dpadUpWasPressed()){
                 b += 15;
             }
             if (gamepad.dpadDownWasPressed()){
                 b -= 15;
             }

        }
        if (equation[Math.abs(stepIndex)].equals("Mid")){
            if (gamepad.dpadUpWasPressed()){
                b50 += 15;
            }
            if (gamepad.dpadDownWasPressed()){
                b50 -= 15;
            }
        }
        if (equation[Math.abs(stepIndex)].equals("High")){
            if (gamepad.dpadUpWasPressed()){
                b100 += 15;
            }
            if (gamepad.dpadDownWasPressed()){
                b100 -= 15;
            }
        }
        calcDistanceLow = (m*usedDistance)+b;
        calcDistanceMid = (m50*usedDistance)+b50;
        calcDistanceHigh = (m100*usedDistance)+b100;

    }

}