package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/*
 * Positive tx = target is to the right of the crosshair
 * Negative tx = target is to the left of the crosshair
 */
public class Limelight {

    private Limelight3A limelight;
    DcMotor turretMotor;
    // The AprilTag / target ID we want to track
    private int desiredTargetId = 24;

    // Proportional gain for turning. Tune this on the field.
    private double kP = 0.015;        // how hard to turn per degree of error
    private double deadbandDeg = 1.0; // if |tx| is smaller than this, we stop
    private double maxPower = 1.0;    // cap turret power so it doesn’t whip


    //the motor that rotates your turret
    public Limelight(HardwareMap Init) {
        turretMotor = Init.get(DcMotor.class, "trrt");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void runLL() {
        // Get the latest result from the Limelight
        LLResult llValue = limelight.getLatestResult();

        // If nothing valid, stop the turret
        if (llValue == null || !llValue.isValid()) {
            aimTurret(0, false, -1);
            return;
        }

        // Horizontal offset from crosshair (in degrees)
        double tx = llValue.getTx();
        int visibleId = desiredTargetId;

        // We have a valid target, so targetVisible = true
        aimTurret(tx, true, visibleId);
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
}
