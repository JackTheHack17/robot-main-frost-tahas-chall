package frc.lib;
import java.lang.Math;
import edu.wpi.first.math.system.plant.DCMotor;

public class TJArmFF {
    public class Joint {
        private double mass;
        private double length;
        private double mg;
        private DCMotor motor;
        private double G = 9.81;

        public Joint(double mass, double length, DCMotor motor) {
            this.mass = mass;
            this.length = length;
            this.mg = mass * G;
            this.motor = motor;
        }

        public double mass() {
            return mass;
        }

        public double length() {
            return length;
        }

        public double mg() {
            return mg;
        }

        public DCMotor motor() {
            return motor;
        }
    }

    private Joint stage1;
    private Joint stage2;
    private Joint stage3;

    public TJArmFF(Joint stage1, Joint stage2, Joint stage3) {
        this.stage1 = stage1;
        this.stage2 = stage2;
        this.stage3 = stage3;
    }

    private double stage1Torque(double theta1) {
        return stage1.mg() * stage1.length()/2 * Math.cos(theta1);
    }

    private double stage2Torque(double theta1, double theta2) {
        return stage1Torque(theta1)
        + 
        stage2.mg() * Math.cos(
            stage1.length * Math.sin(theta1)
            + 
            stage2.length/2 * Math.sin(theta2)
        )
        *
        Math.sqrt(
            Math.pow(
                stage1.length * Math.cos(theta1) + stage2.length/2 * Math.cos(theta2), 2)
            +
                Math.pow(
                    stage1.length * Math.sin(theta1) + stage2.length/2 * Math.sin(theta2), 2)
        )
        ;
    }

    private double stage3Torque(double theta1, double theta2, double theta3) {
        return stage2Torque(theta1, theta2)
        +
        stage3.mg() * Math.cos(
            stage1.length * Math.sin(theta1)
            + 
            stage2.length * Math.sin(theta2)
            + 
            stage3.length/2 * Math.sin(theta3)
        )
        *
        Math.sqrt(
            Math.pow(
                stage1.length * Math.cos(theta1) + stage2.length 
                * 
                Math.cos(theta2) + stage3.length/2 * Math.cos(theta3), 2)
            +
                Math.pow(
                    stage1.length * Math.sin(theta1) + stage2.length 
                    * 
                    Math.sin(theta2) + stage3.length/2 * Math.sin(theta3), 2)
        );
    }

    public double calculateStage1(double theta1Radians, double kS, double vRadiansPerSecond) {
        return stage1.motor.getVoltage(stage1Torque(theta1Radians), 0) 
        + 
        kS * Math.signum(vRadiansPerSecond);
    }

    public double calculateStage2(double theta1Radians, double theta2Radians, double kS, double vRadiansPerSecond) {
        return stage2.motor.getVoltage(stage2Torque(theta1Radians, theta2Radians), 0) 
        + 
        kS * Math.signum(vRadiansPerSecond);
    }

    public double calculateStage3(double theta1Radians, double theta2Radians, double theta3Radians, double kS, double vRadiansPerSecond) {
        return stage3.motor.getVoltage(stage3Torque(theta1Radians, theta2Radians, theta3Radians), 0) 
        + 
        kS * Math.signum(vRadiansPerSecond);
    }
}