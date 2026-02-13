package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TurretTesting {

    private Limelight3A limelight;
    private DcMotor turretMotor;
    private IMU imu;

    // Debug: store the latest Limelight result so we can report validity/tx to TeleOp
    private LLResult lastResult = null;

    // Tunable constants
    // Close-range oscillation killers (Limelight tx is noisier when close)
    private double visionDeadbandDeg = 0.8; // ignore tiny tx jitter
    private double closeGainK = 0.45;      // scale P gain by 1/(1 + closeGainK*ta)
    private double minPowerCloseScale = 0.6; // reduce minPower when close (0..1)

    private double kP = 0.047;         // proportional gain (deg -> power)
    private double kD = 0.0015;        // derivative gain (power per (deg/s))
    private double lastErrDeg = 0.0;  // for derivative
    private double deadbandDeg = 0.9; // stop when |tx| < deadband
    private double maxPower = 0.85;   // clamp output (allow faster catch-up)
    private double minPower = 0.10;   // minimum power to overcome stiction (applied only when far off)
    private double minPowerErrDeg = 2.0; // only enforce minPower when |tx| > this

    // Direction flip: if turret runs away / turns the wrong way, set this to -1
    private int turretSign = 1;

    // Smoothing: low-pass filter for tx (reduces jitter/frame stepping)
    private double txAlpha = 0.45;   // 0..1 (higher = faster response; still smooth)
    private double filteredTx = 0.0;

    // Smoothing: slew-rate limit for motor power changes (reduces jerkiness)
    private double maxPowerSlewPerSec = 10.0; // base slew (power units per second)

    // When the robot yaws fast, allow faster slew smoothly (prevents losing the tag on spins)
    private double maxSlewPerSec = 18.0;      // cap
    private double yawSlewGain = 0.03;        // adds slew per (deg/s). Example: 150 deg/s -> +4.5

    // Slew limiter state (remember last commanded power)
    private double lastPowerCmd = 0.0;

    private long lastLoopNs = 0;

    // IMU feedforward: helps turret keep up while the robot yaws (turns in place)
    private double kFF = 0.010;           // (deg/s -> power) stronger for fast spins; tune on field
    private double yawRateDeadzone = 4.0; // ignore small IMU drift/noise (deg/s)

    // Filter yaw rate to prevent feedforward jitter
    private double yawAlpha = 0.25;       // 0..1 (higher = less filtering)
    private double filteredYawRate = 0.0; // deg/s

    // --- OPTIONAL gyro angle stabilization (field-oriented turret) ---
    private boolean useGyroAngleHold = false; // OFF by default
    private double lastYawDeg = 0.0;
    private double ticksPerDegree = 6090.0 / 360.0; // match your turret gearing

    // Turret encoder soft limits (no slip ring)
    private int maxTurretTicks = 754;
    private int minTurretTicks = -745;

    // Soft-limit buffer so we stop a little early to prevent overshoot/coast
    private int limitBufferTicks = 15;

    public TurretTesting(HardwareMap hw) {
        turretMotor = hw.get(DcMotor.class, "trrt");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hw.get(Limelight3A.class, "limelight");

        // IMU (used for feedforward while rotating)
        imu = hw.get(IMU.class, "imu");

        limelight.start();

        lastLoopNs = System.nanoTime();
    }

    public void setGains(double kP, double deadbandDeg, double maxPower) {
        this.kP = kP;
        this.deadbandDeg = deadbandDeg;
        this.maxPower = Math.abs(maxPower);

        // Reset smoothing state so changes apply cleanly
        filteredTx = 0.0;
        lastPowerCmd = 0.0;
        lastLoopNs = System.nanoTime();
    }

    public void setPD(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
        this.filteredTx = 0.0;
        this.lastErrDeg = 0.0;
        this.lastPowerCmd = 0.0;
        this.lastLoopNs = System.nanoTime();
    }

    /* Set the Limelight pipeline explicitly (use the same pipeline number you see in the web UI). */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /** Flip turret direction quickly. Pass +1 or -1. */
    public void setTurretSign(int sign) {
        turretSign = (sign >= 0) ? 1 : -1;
    }

    /** Tune smoothing. txAlpha: 0..1 (higher = faster). slewPerSec: 2..6 typical. */
    public void setSmoothing(double txAlpha, double slewPerSec) {
        this.txAlpha = Math.max(0.0, Math.min(1.0, txAlpha));
        this.maxPowerSlewPerSec = Math.max(0.1, slewPerSec);
    }

    /** Tune stiction handling. */
    public void setStiction(double minPower, double minPowerErrDeg) {
        this.minPower = Math.abs(minPower);
        this.minPowerErrDeg = Math.abs(minPowerErrDeg);
    }

    /** Tune close-range stability. closeGainK reduces aggressiveness as ta grows (closer target). */
    public void setCloseRangeTuning(double visionDeadbandDeg, double closeGainK, double minPowerCloseScale) {
        this.visionDeadbandDeg = Math.max(0.0, visionDeadbandDeg);
        this.closeGainK = Math.max(0.0, closeGainK);
        this.minPowerCloseScale = Math.max(0.0, Math.min(1.0, minPowerCloseScale));
    }

    /** Tune IMU feedforward: kFF converts yaw rate (deg/s) to motor power. */
    public void setFeedforward(double kFF, double yawRateDeadzone) {
        this.kFF = kFF;
        this.yawRateDeadzone = Math.abs(yawRateDeadzone);
    }

    /** Tune yaw filtering + yaw-based slew adaptation. */
    public void setYawTuning(double yawAlpha, double yawSlewGain, double maxSlewPerSec) {
        this.yawAlpha = Math.max(0.0, Math.min(1.0, yawAlpha));
        this.yawSlewGain = Math.max(0.0, yawSlewGain);
        this.maxSlewPerSec = Math.max(this.maxPowerSlewPerSec, Math.abs(maxSlewPerSec));
    }

    /** Enable/disable gyro angle hold (field-oriented turret). */
    public void setGyroAngleHold(boolean enabled) {
        this.useGyroAngleHold = enabled;
        lastYawDeg = getRobotYawDeg();
    }

    /** If your encoder ratio changes, update this. */
    public void setTicksPerDegree(double ticksPerDegree) {
        this.ticksPerDegree = ticksPerDegree;
    }

    public void setTurretLimits(int minTicks, int maxTicks, int bufferTicks) {
        this.minTurretTicks = minTicks;
        this.maxTurretTicks = maxTicks;
        this.limitBufferTicks = Math.max(0, bufferTicks);
    }

    /* Fetch and store the latest Limelight result (no turret movement). Call every loop for live telemetry. */
    public void updateLL() {
        lastResult = limelight.getLatestResult();
    }

    /* Call once per loop */
    public void runLL() {
        updateLL();
        LLResult r = lastResult;

        // Optional gyro angle hold: counter-rotate turret based on robot yaw angle
        if (useGyroAngleHold && imu != null) {
            double yawNow = getRobotYawDeg();
            double dYaw = wrapDeg(yawNow - lastYawDeg);
            lastYawDeg = yawNow;

            // Counter-rotate turret setpoint implicitly by biasing filteredTx
            filteredTx -= dYaw; // degrees
        }

        // No valid target → stop + reset smoothing so it doesn't "step" on reacquire
        if (r == null || !r.isValid()) {
            turretMotor.setPower(0);
            filteredTx = 0.0;
            lastPowerCmd = 0.0;
            filteredYawRate = 0.0;
            lastErrDeg = 0.0;
            lastLoopNs = System.nanoTime();
            if (useGyroAngleHold && imu != null) lastYawDeg = getRobotYawDeg();
            return;
        }

        double tx = r.getTx();
        double ta = r.getTa(); // target area (bigger when closer)

        // Ignore small tx jitter (especially noticeable at close range)
        if (Math.abs(tx) < visionDeadbandDeg) {
            tx = 0.0;
        }

        // Low-pass filter tx for smoother tracking (reduces jitter / frame stepping)
        filteredTx = filteredTx + txAlpha * (tx - filteredTx);

        // Scale vision aggressiveness down when close (ta increases when close)
        // This directly reduces oscillation when you're near the tag.
        double kPScaled = kP / (1.0 + closeGainK * Math.max(0.0, ta));

        // Deadband
        if (Math.abs(filteredTx) < deadbandDeg) {
            turretMotor.setPower(0);
            lastPowerCmd = 0.0;
            filteredYawRate = 0.0;
            lastErrDeg = 0.0;
            lastLoopNs = System.nanoTime();
            return;
        }

        // Slew-rate limit motor command for smoothness (dt calculation moved up for D term)
        long nowNs = System.nanoTime();
        double dt = (lastLoopNs == 0) ? 0.02 : (nowNs - lastLoopNs) / 1e9; // seconds
        lastLoopNs = nowNs;

        // Safety clamp dt in case of a pause / breakpoint
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        // IMU feedforward to keep up while the robot is rotating
        double ffPower = 0.0;
        double absYawRate = 0.0;

        if (imu != null) {
            AngularVelocity vel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double yawRate = vel.zRotationRate; // deg/s

            // Low-pass filter yaw rate to avoid jittery feedforward
            filteredYawRate = filteredYawRate + yawAlpha * (yawRate - filteredYawRate);

            // Deadzone on the filtered value
            if (Math.abs(filteredYawRate) < yawRateDeadzone) {
                filteredYawRate = 0.0;
            }

            absYawRate = Math.abs(filteredYawRate);

            // Negative sign counters robot yaw (flip turretSign if your direction is reversed)
            ffPower = -kFF * filteredYawRate;
        }

        // Derivative on error (deg/s) for damping (use filteredTx so it isn't noisy)
        double dErrDegPerSec = 0.0;
        if (dt > 0.0) {
            dErrDegPerSec = (filteredTx - lastErrDeg) / dt;
        }
        lastErrDeg = filteredTx;

        // PD on tx + feedforward (apply direction flip)
        double power = turretSign * (kPScaled * filteredTx + kD * dErrDegPerSec + ffPower);

        // Stiction help only when we're meaningfully off-target (prevents jitter near center)
        // Reduce minPower when close so we don't "buzz" around the target.
        double minPowerScaled = minPower;
        if (ta > 0.0) {
            minPowerScaled = minPower * (1.0 / (1.0 + minPowerCloseScale * ta));
        }

        if (Math.abs(filteredTx) > minPowerErrDeg && Math.abs(power) < minPowerScaled) {
            power = Math.signum(power) * minPowerScaled;
        }

        // Clamp
        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        // Enforce turret encoder soft limits
        power = applySoftLimits(power);

        // Smoothly increase allowed slew as robot yaw rate increases
        double slewPerSec = maxPowerSlewPerSec + yawSlewGain * absYawRate;
        if (slewPerSec > maxSlewPerSec) slewPerSec = maxSlewPerSec;

        double maxDelta = slewPerSec * dt;
        double delta = power - lastPowerCmd;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;

        lastPowerCmd += delta;
        turretMotor.setPower(lastPowerCmd);
    }

    private double getRobotYawDeg() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    private double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    /** Apply encoder soft limits: if we're at/near a limit, block power that would push further into it. */
    private double applySoftLimits(double power) {
        int pos = turretMotor.getCurrentPosition();

        // Near/at max limit: block positive power (would increase ticks)
        if (pos >= (maxTurretTicks - limitBufferTicks) && power > 0) {
            return 0.0;
        }

        // Near/at min limit: block negative power (would decrease ticks)
        if (pos <= (minTurretTicks + limitBufferTicks) && power < 0) {
            return 0.0;
        }

        return power;
    }

    public void stopTurret() {
        turretMotor.setPower(0);
        filteredTx = 0.0;
        lastPowerCmd = 0.0;
        filteredYawRate = 0.0;
        lastErrDeg = 0.0;
        lastLoopNs = System.nanoTime();
    }

    /* True if Limelight has a valid target in the current pipeline (AprilTag pipeline, etc.). */
    public boolean hasValidTarget() {
        return lastResult != null && lastResult.isValid();
    }

    /* True if getLatestResult() returned null this loop (no packet/result yet). */
    public boolean hasResultPacket() {
        return lastResult != null;
    }

    /*returns true if we have a result packet and it's valid. */
    public boolean isResultValid() {
        return lastResult != null && lastResult.isValid();
    }

    /* Latest horizontal offset (tx). Returns 0 if there is no valid target. */
    public double getTx() {
        if (lastResult == null || !lastResult.isValid()) return 0.0;
        return lastResult.getTx();
    }

    /* Returns a short status string you can print in telemetry. */
    public String getLimelightStatus() {
        if (lastResult == null) return "NO_RESULT";
        if (!lastResult.isValid()) return "INVALID";
        return "VALID";
    }
}