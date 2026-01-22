package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.Units;
import frc.robot.Hardware;

public class IntakeIOImp implements IntakeIO {

    private final TalonFX deployMotor;
    private final TalonFX rollerMotor;
    private final DigitalInput beamSensor;

    // Track last commanded deploy state; IO doesn't try to read a position encoder here.
    private boolean deployed = false;

    public IntakeIOImp() {
        deployMotor = new TalonFX(Hardware.INTAKE_DEPLOY_MOTOR_ID);
        rollerMotor = new TalonFX(Hardware.INTAKE_SPINNY_MOTOR_ID);
        beamSensor = new DigitalInput(Hardware.INTAKE_SENSOR);

        configureMotor(deployMotor, false, 20, 10);
        configureMotor(rollerMotor, false, 40, 20);
    }

    private void configureMotor(TalonFX motor, boolean inverted, double statorCurrentLimit, double supplyCurrentLimit) {
        TalonFXConfigurator cfg = motor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Conservative current limits
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.apply(talonFXConfiguration);
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    @Override
    public void setDeployed(boolean deployed) {
        // Simple open-loop deploy/retract command. Tune voltages as needed.
        if (deployed) {
            deployMotor.setControl(new VoltageOut(6.0));
        } else {
            deployMotor.setControl(new VoltageOut(-6.0));
        }
        this.deployed = deployed;
    }

    @Override
    public boolean isDeployed() {
        return deployed;
    }

    @Override
    public boolean hasGamePiece() {
        // Assumption: sensor.get() == true when piece present. If your sensor is
        // active-low (common), return !beamSensor.get() instead.
        return beamSensor.get();
    }

    @Override
    public double getRollerCurrent() {
        return rollerMotor.getSupplyCurrent().getValue().in(Units.Amps);
    }
}
