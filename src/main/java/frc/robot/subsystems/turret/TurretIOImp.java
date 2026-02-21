package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.units.Units;
import frc.robot.Hardware;

public class TurretIOImp implements TurretIO {

    // Constants
    private static final double GEAR_RATIO = 15.0;
    private static final double CANCODER1_GEAR_RATIO = 25.0;
    private static final double CANCODER2_GEAR_RATIO = 26.0;
    private static final double KP = 1.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double KS = 0.0;
    private static final double KV = 0.0;
    private static final double KA = 0.0;
    private static final double KG = 0.0; // Unused for pivots
    private static final boolean BRAKE_MODE = true;
    private static final boolean ENABLE_STATOR_LIMIT = true;
    private static final double STATOR_CURRENT_LIMIT = 20.0;
    private static final boolean ENABLE_SUPPLY_LIMIT = true;
    private static final double SUPPLY_CURRENT_LIMIT = 10.0;

    // Feedforward
    private final ArmFeedforward feedforward = new ArmFeedforward(
        KS, // kS
        0, // kG - Pivot doesn't need gravity compensation
        KV, // kV
        KA // kA
    );

    // Motor controller
    private final TalonFX motor;

    // Simulation
    private final SingleJointedArmSim pivotSim;

    public TurretIOImp() {
        motor = new TalonFX(Hardware.TURRET_MOTOR_ID);
        factoryDefaults();

        // Initialize simulation
        pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            GEAR_RATIO,
            0.01, // Arm moment of inertia - Small value since there are no arm parameters
            0.1, // Arm length (m) - Small value since there are no arm parameters
            Degrees.of(-90).in(Units.Radians), // Min angle (rad)
            Degrees.of(90).in(Units.Radians), // Max angle (rad)
            false, // Simulate gravity - Disable gravity for pivot
            Degrees.of(0).in(Units.Radians) // Starting position (rad)
        );
    }

    // TalonFX config
    private void factoryDefaults() {
        TalonFXConfigurator cfg = motor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        // Inverting motor output direction
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // Setting the motor to brake when not moving
        talonFXConfiguration.MotorOutput.NeutralMode = BRAKE_MODE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        // remote sensor values for the WCP encoder
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = Hardware.TURRET_CANCODER_ID1;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // previously mentioned ratio calculation
        talonFXConfiguration.Feedback.RotorToSensorRatio = CANCODER1_GEAR_RATIO;
        
        // enabling current limits
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = ENABLE_STATOR_LIMIT;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = ENABLE_SUPPLY_LIMIT;

        // PID
        // set slot 0 gains
        talonFXConfiguration.Slot0.kS = KS;
        talonFXConfiguration.Slot0.kV = KV;
        talonFXConfiguration.Slot0.kA = KA;
        talonFXConfiguration.Slot0.kP = KP;
        talonFXConfiguration.Slot0.kI = KI;
        talonFXConfiguration.Slot0.kD = KD;
        talonFXConfiguration.Slot0.kG = KG;
        talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // set Motion Magic settings in rps not mechanism units
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 320; // 160 // Target cruise velocity of 2560 rps
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 320; // Target acceleration of 4960 rps/s (0.5
                                                                        // seconds)
        talonFXConfiguration.MotionMagic.MotionMagicJerk = 3200; // 1600 // Target jerk of 6400 rps/s/s (0.1 seconds)

        cfg.apply(talonFXConfiguration);
    }

    @Override
    public SingleJointedArmSim getSimulation() {
        return pivotSim;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public double getPositionRadians() {
        return motor.getPosition().getValue().in(Units.Radians);
    }

    @Override
    public double getVelocityRadiansPerSec() {
        return motor.getVelocity().getValue().in(Units.RadiansPerSecond);
    }

    @Override
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValue().in(Units.Amps);
    }
    
}
