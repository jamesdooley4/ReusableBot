package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        MOVING_TO_POSE,
        MANUAL
    }

    public static class ArmPose {
        public final double shoulderRad;
        public final double elbowRad;

        public ArmPose(double shoulderRad, double elbowRad) {
            this.shoulderRad = shoulderRad;
            this.elbowRad = elbowRad;
        }
    }

    private final ArmIO io;
    private State state = State.IDLE;
    private ArmPose targetPose = new ArmPose(0.0, 0.0);

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    public void moveToPose(ArmPose pose) {
        state = State.MOVING_TO_POSE;
        targetPose = pose;
        io.setShoulderAngleRadians(pose.shoulderRad);
        io.setElbowAngleRadians(pose.elbowRad);
    }

    public void manualControl(double shoulderVolts, double elbowVolts) {
        state = State.MANUAL;
        io.setShoulderVoltage(shoulderVolts);
        io.setElbowVoltage(elbowVolts);
    }

    public boolean atPose(double toleranceRad) {
        return Math.abs(io.getShoulderAngleRadians() - targetPose.shoulderRad) <= toleranceRad
            && Math.abs(io.getElbowAngleRadians() - targetPose.elbowRad) <= toleranceRad;
    }

    public void stop() {
        state = State.IDLE;
        io.setShoulderVoltage(0.0);
        io.setElbowVoltage(0.0);
    }

    public State getState() {
        return state;
    }

    // --- Command factories ---

    /**
     * Move arm to a pose and finish when within tolerance.
     */
    public Command moveToPoseCommand(ArmPose pose, double toleranceRad) {
        return Commands.sequence(
            runOnce(() -> moveToPose(pose)),
            // wait until at pose
            run(() -> {}).until(() -> atPose(toleranceRad)),
            runOnce(() -> stop())
        );
    }

    /** Manual control using separate suppliers for shoulder and elbow. */
    public Command manualControlCommand(DoubleSupplier shoulderSupplier, DoubleSupplier elbowSupplier) {
        return run(() -> manualControl(shoulderSupplier.getAsDouble(), elbowSupplier.getAsDouble()));
    }
}
