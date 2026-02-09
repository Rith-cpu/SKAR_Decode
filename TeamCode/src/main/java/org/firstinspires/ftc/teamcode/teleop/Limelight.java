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
    // Desired AprilTag ID to track (NOTE: not enforced until we parse tag IDs from LLResult)
    private int desiredTargetId = 24;

    // Baseline P controller (tx is in degrees)
    // NOTE: 0.0001 is effectively zero for tx-in-degrees; use something in the ~0.02–0.08 range.
    private double kP = 0.04;          // power per degree of tx
    private double deadbandDeg = 0.7;  // stop when |tx| < deadband
    private double maxPower = 0.7;     // clamp output
    private double minPower = 0.0;     // set to 0 to avoid buzzing; only enable if you truly have stiction

    // Smoothing: filter tx and slew-limit power so rotation is smooth
    private double txAlpha = 0.35;        // 0..1 (higher = faster response)
    private double filteredTx = 0.0;

    private double maxPowerSlewPerSec = 6.0; // power units per second
    private double lastPowerCmd = 0.0;
    private long lastLoopNs = 0;

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
        lastLoopNs = System.nanoTime();
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

        // Smooth tx so the turret doesn't "step" frame-to-frame
        if (targetVisible) {
            filteredTx = filteredTx + txAlpha * (tx - filteredTx);
        } else {
            filteredTx = 0.0;
        }

        aimTurret(filteredTx, targetVisible);
    }

    /*
     * Main method
     * @param tx  horizontal offset from Limelight (degrees)
     * @param targetVisible true if Limelight currently sees any valid target
     */
    public void aimTurret(double tx, boolean targetVisible) {

        if (!targetVisible) {
            turretMotor.setPower(0);
            lastPowerCmd = 0.0;
            lastLoopNs = System.nanoTime();
            return;
        }

        // If lined up, then stop
        if (Math.abs(tx) < deadbandDeg) {
            turretMotor.setPower(0);
            lastPowerCmd = 0.0;
            lastLoopNs = System.nanoTime();
            return;
        }

        // Proportional control
        double power = kP * tx;

        // Only apply minPower when we're meaningfully off-target; otherwise it causes buzzing.
        if (minPower > 0.0 && Math.abs(tx) > deadbandDeg * 2.0 && Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        // Clamp power
        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        // Slew-rate limit motor command for smoothness
        long nowNs = System.nanoTime();
        double dt = (lastLoopNs == 0) ? 0.02 : (nowNs - lastLoopNs) / 1e9;
        lastLoopNs = nowNs;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double maxDelta = maxPowerSlewPerSec * dt;
        double delta = power - lastPowerCmd;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;

        lastPowerCmd += delta;
        turretMotor.setPower(lastPowerCmd);
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