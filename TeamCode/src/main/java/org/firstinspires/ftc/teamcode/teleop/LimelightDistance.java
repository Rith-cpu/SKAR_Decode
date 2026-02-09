package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightDistance {

    private Limelight3A limelight;

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
        double m = 3.33499;
        double b = 1061.85831; // 1071.85831 new -10
        return (m*distance)+b;
    }

    public Double getHighPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceGreaterThan100(distance);
    }

    public double estimatePowerFromDistanceLessThan100GreaterThan50(double distance){
        double m = 4.6945;
        double b = 983.67251; // 991.50633, 980.50633
        return m*distance+b;
    }

    public Double getLowPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceLessThan100GreaterThan50(distance);
    }

    public double estimatePowerFromDistanceLessThan50(double distance){
        double m = 4.82433;
        double b = 973.16097; // 0.3669
        return m*distance+b;
    }

    public Double getLowLowPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceLessThan50(distance);
    }

}
