package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Positive tx = target is to the right of the crosshair
 * Negative tx = target is to the left of the crosshair
 */
public class Limelight {

    private Limelight3A limelight;
    private DcMotor turretMotor;
    private LLResult lastResult = null;
    // The AprilTag / target ID we want to track
    private int desiredTargetId = 24;

    // Proportional gain for turning. Tune this on the field.
    private double kP = 0.04;         // more aggressive: turn harder per degree of error
    private double deadbandDeg = 1.0; // if |tx| is smaller than this, we stop
    private double maxPower = 0.8;    // allow the turret to move fast but not full send
    private double minPower = 0.12;   // minimum power to overcome static friction

    // Simple state to carry over between frames so we don't freeze when a frame is missed
    private double lastTx = 0.0;
    private boolean lastVisible = false;
    private long lastUpdateMs = 0;

    //the motor that rotates your turret
    public Limelight(HardwareMap Init) {
        turretMotor = Init.get(DcMotor.class, "trrt");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight = Init.get(Limelight3A.class, "limelight");
        limelight.start();
    }


    /*
     * Set which target ID (e.g., AprilTag ID) the turret should follow.
     */
    public void setTargetId(int id) {
        this.desiredTargetId = id;
    }


    public void setGains(double kP, double deadbandDeg, double maxPower) {
        this.kP = kP;
        this.deadbandDeg = deadbandDeg;
        this.maxPower = Math.abs(maxPower);
    }

    public void setGains(double kP, double deadbandDeg, double maxPower, double minPower) {
        this.kP = kP;
        this.deadbandDeg = deadbandDeg;
        this.maxPower = Math.abs(maxPower);
        this.minPower = Math.abs(minPower);
    }

    public void runLL() {
        // Get the latest result from the Limelight
        LLResult llValue = limelight.getLatestResult();
        lastResult = llValue;
        long now = System.currentTimeMillis();

        double tx;
        boolean targetVisible;

        if (llValue != null && llValue.isValid()) {
            // Fresh valid frame
            tx = llValue.getTx();
            targetVisible = true;

            // Save as last good state
            lastTx = tx;
            lastVisible = true;
            lastUpdateMs = now;
        } else {
            // No new valid frame.
            // Reuse the last one for a short time window so the turret doesn't freeze
            if (lastVisible && (now - lastUpdateMs) < 200) { // 200 ms hold
                tx = lastTx;
                targetVisible = true;
            } else {
                tx = 0.0;
                targetVisible = false;
            }
        }

        int visibleId = desiredTargetId;
        aimTurret(tx, targetVisible, visibleId);
    }

    /*
     * Main method
     * @param tx  horizontal offset from Limelight (degrees)
     * @param targetVisible true if Limelight currently sees any valid target
     * @param visibleId     the ID of the target currently seen (e.g., AprilTag ID)
     */
    public void aimTurret(double tx, boolean targetVisible, int visibleId) {

        if (!targetVisible || (desiredTargetId != -1 && visibleId != desiredTargetId)) {
            turretMotor.setPower(0);
            return;
        }

        // If lined up, then stop
        if (Math.abs(tx) < deadbandDeg) {
            turretMotor.setPower(0);
            return;
        }

        // Proportional control
        double power = kP * tx;

        // Apply a minimum power to overcome static friction so small errors still move
        if (Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        // Clamp power to a safe range.
        if (power > maxPower) {
            power = maxPower;
        } else if (power < -maxPower) {
            power = -maxPower;
        }

        turretMotor.setPower(power);
    }


    public void stopTurret() {
        turretMotor.setPower(0);
    }

    /** True when Limelight is returning a valid result this loop. */
    public boolean isTargetValid() {
        return lastResult != null && lastResult.isValid();
    }

    /** Horizontal offset (tx) in degrees. Returns 0 when not valid. */
    public double getTx() {
        if (lastResult == null || !lastResult.isValid()) return 0.0;
        return lastResult.getTx();
    }
}
