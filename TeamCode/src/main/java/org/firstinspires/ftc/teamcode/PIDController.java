package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double targetAngle;
    private double Kp = 0.0;
    private double Ki = 0.0;
    private double Kd = 0.0;

    private  double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;

    private double lastTime = 0.0;
    private double lastSlope = 0.0;

    public PIDController(double target, double p, double i, double d) {
        this.targetAngle = target;
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    public double update(double currentAngle) {
        // Calculate Proportional (P)
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        // Calculate Integral (I)
        accumulatedError = accumulatedError + error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // Calculate Derivative (D)
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        // Calculate Motor Power
        double motorPower = 0.1 * Math.signum(error)
                + 0.9 * Math.tanh(Kp * error + Ki * accumulatedError + Kd * slope);
        return motorPower;
    }

    public double getLastSlope() {
        return this.lastSlope;
    }
}
