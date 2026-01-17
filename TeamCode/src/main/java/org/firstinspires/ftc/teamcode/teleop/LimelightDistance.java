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
        double m = 0.00312311;
        double b = 0.246143; // 0.246143
        return (m*distance)+b;
    }

    public Double getHighPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceGreaterThan100(distance);
    }

    public double estimatePowerFromDistanceLessThan100GreaterThan50(double distance){
        double m = 0.002067;
        double b = 0.3978; // 0.3978
        return m*distance+b;
    }

    public Double getLowPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceLessThan100GreaterThan50(distance);
    }

    public double estimatePowerFromDistanceLessThan50(double distance){
        double m = 0.003001;
        double b = 0.3669; // 0.3669
        return m*distance+b;
    }

    public Double getLowLowPowerIfValid(double distance) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;
        return estimatePowerFromDistanceLessThan50(distance);
    }

}
