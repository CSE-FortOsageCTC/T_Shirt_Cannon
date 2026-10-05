package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class AutoRotateUtil {

    private final PIDController pidController;

    private double m_angle;

    public AutoRotateUtil(double angle) {

        this.m_angle = angle == 0?360:angle;

        this.pidController = new PIDController(0, 0, 0);

        pidController.setTolerance(0.001);
        pidController.setSetpoint(0);
   }

   public void initialize() {
    pidController.reset();
   }

   public void reset() {
    pidController.reset();
   }


   public double calculateRotationSpeed () {

    this.pidController.setP(.01);
    this.pidController.setI(0);
    this.pidController.setD(0);

    double headingError = this.m_angle % 360;
    if (headingError > 180) {
        headingError -= 360;
    }
    if (headingError < -180) {
        headingError += 360;
    }

    double feedForward = 0.5;

    if (Math.abs(headingError) > Constants.feedForwardAngle) {
        return (headingError < 0) ? feedForward : -feedForward;
    } else {
        return MathUtil.clamp(pidController.calculate(headingError, 0), -1, 1);
    }
   }
   /**
    * Updates degrees robot needs to rotate
    */ 
   public void updateTargetAngle(double angle) {
    m_angle = angle;

   }

   public boolean isFinished () {
    return pidController.atSetpoint();
   }

   public void end() {
    System.out.println("it ended");
    pidController.reset();
   }

}
