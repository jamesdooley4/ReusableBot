package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc.robot.Hardware;

public class LauncherIOImp implements LauncherIO {

    private final TalonFX flywheelLeft;
    private final TalonFX flywheelRight;
    private final TalonFX hoodMotor;

    // Simple gains for hood position (open-loop-ish)
    // (Using Talon native MotionMagicVoltage for hood position now)

    public LauncherIOImp() {
        flywheelLeft = new TalonFX(Hardware.LAUNCHER_MOTOR_LEFT_ID);
        flywheelRight = new TalonFX(Hardware.LAUNCHER_MOTOR_RIGHT_ID);
        // Dedicated hood motor
        hoodMotor = new TalonFX(Hardware.LAUNCHER_HOOD_MOTOR_ID);

        // Configure flywheel motors
        configureFlywheelMotor(flywheelLeft, false);
        configureFlywheelMotor(flywheelRight, true);

        // Configure hood motor for position (MotionMagic)
        configureHoodMotor(hoodMotor, false);
    }

    private void configureFlywheelMotor(TalonFX motor, boolean inverted) {
        TalonFXConfigurator cfg = motor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Basic current limits
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60.0;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40.0;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID (slot 0)
        talonFXConfiguration.Slot0.kP = 0.002; // small P for velocity
        talonFXConfiguration.Slot0.kI = 0.0;
        talonFXConfiguration.Slot0.kD = 0.0;
        talonFXConfiguration.Slot0.kV = 0.0;
        talonFXConfiguration.Slot0.kS = 0.0;

        cfg.apply(talonFXConfiguration);
    }

    private void configureHoodMotor(TalonFX motor, boolean inverted) {
        TalonFXConfigurator cfg = motor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // MotionMagic settings (tune as needed)
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 50; // sensor units/s (tunable)
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 100; // sensor units/s^2
        talonFXConfiguration.MotionMagic.MotionMagicJerk = 1000;

        // Position PID for hood
        talonFXConfiguration.Slot0.kP = 4.0;
        talonFXConfiguration.Slot0.kI = 0.0;
        talonFXConfiguration.Slot0.kD = 0.0;

        // Current limits for hood
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 20.0;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.apply(talonFXConfiguration);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelLeft.setControl(new VoltageOut(volts));
        flywheelRight.setControl(new VoltageOut(volts));
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        // Use TalonFX native velocity control if available (set in rad/s)
        double desiredRadPerSec = rpm * 2.0 * Math.PI / 60.0;
        // Use VelocityVoltage (native) to command closed-loop velocity on the Talon
        flywheelLeft.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(desiredRadPerSec));
        flywheelRight.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(desiredRadPerSec));
    }

    @Override
    public void stopFlywheel() {
        flywheelLeft.stopMotor();
        flywheelRight.stopMotor();
    }

    @Override
    public void setHoodPositionRadians(double radians) {
        // Use TalonFX MotionMagic position control (native) if available
        // Command the motor with a MotionMagic position target in radians
        hoodMotor.setControl(new com.ctre.phoenix6.controls.MotionMagicVoltage(radians));
    }

    @Override
    public double getHoodPositionRadians() {
        return hoodMotor.getPosition().getValue().in(Units.Radians);
    }

    @Override
    public double getFlywheelVelocityRPM() {
        double radPerSec = getFlywheelVelocityRadPerSec();
        return radPerSec * 60.0 / (2.0 * Math.PI);
    }

    public double getFlywheelVelocityRadPerSec() {
        return flywheelLeft.getVelocity().getValue().in(Units.RadiansPerSecond);
    }

    @Override
    public double getFlywheelCurrent() {
        return (flywheelLeft.getSupplyCurrent().getValue().in(Units.Amps)
                + flywheelRight.getSupplyCurrent().getValue().in(Units.Amps)) / 2.0;
    }

}
