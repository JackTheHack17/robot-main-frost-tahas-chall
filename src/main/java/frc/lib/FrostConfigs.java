package frc.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.DRIVETRAIN.*;

import edu.wpi.first.math.controller.PIDController;

public class FrostConfigs {
    public static void configDrive (TalonFX motor) {
        configDrive(motor, DRIVE_kP, DRIVE_kF);
    }
    
    public static void configAzimuth (TalonFX motor, CANCoder position) {
        configAzimuth(motor, position, AZIMUTH_kP, AZIMUTH_kD, AZIMUTH_kF);
    }
    
    public static void configDrive (TalonFX motor, double kP, double kF) {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
            true, 
            60, 
            60, 
            0));
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.setSelectedSensorPosition(0);
        motor.config_kP(0, kP);
        motor.config_kF(0, kF);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
      }
    
    public static void configAzimuth (TalonFX motor, CANCoder position, double kP, double kD, double kF) {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configRemoteFeedbackFilter(position, 0);
        motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        motor.configSelectedFeedbackCoefficient(360 / (2048 * AZIMUTH_GEAR_RATIO));
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
            true, 
            25, 
            40, 
            0));
        motor.setSelectedSensorPosition(degreesToFalcon(position.getAbsolutePosition()));
        motor.config_kP(0, kP);
        motor.config_kD(0, kD);
        motor.config_kF(0, kF);
        motor.configNeutralDeadband(AZIMUTH_DEADBAND);
      }
    
    public static void configPosition (CANCoder encoder, double offset) {
        encoder.configFactoryDefault();
        encoder.configMagnetOffset(offset);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.setPositionToAbsolute();
      }
    
    public static void configAzimuthPID(PIDController controller) {
        controller.enableContinuousInput(0, 360);
        controller.setTolerance(0);
    }

    public static double degreesToFalcon(double degrees) {
        return degrees / (360.0 / ( AZIMUTH_GEAR_RATIO * 2048.0));
    }

    public static void configIntakeMotor(CANSparkMax motor, boolean invert) {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setInverted(invert);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(20);
        motor.setSecondaryCurrentLimit(20);
        motor.setCANTimeout(20);
        motor.burnFlash();
    }

    public static void configArmMotor(CANSparkMax motor, boolean invert) {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setInverted(invert);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.burnFlash();
    }       
}