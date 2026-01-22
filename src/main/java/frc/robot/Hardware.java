package frc.robot;

public class Hardware {
  // Add motor IDs here

  public static final int PDH_ID = 1;
  // Swerve: 1-12

  // LEDs [15-19]
  public static final int LED_ID = 15;

  // index [20-24]
  public static final int INDEX_SPINNY_MOTOR = 20;
  public static final int INDEX_SENSOR = 21;

  // wrist [25-29]
  public static final int WRIST_MOTOR_ID = 30;
  public static final int WRIST_CANDI_ID = 31;

  // arm pivot [30-34]
  public static final int ARM_PIVOT_MOTOR_ID = 30;
  public static final int ARM_PIVOT_CANDI_ID = 31;
  public static final int MAIN_ARM_SENSOR = 32;

  // turret [35-39]
  public static final int TURRET_MOTOR_ID = 35;
  public static final int TURRET_CANCODER_ID1 = 36;
  public static final int TURRET_CANCODER_ID2 = 37;

  // gripper [40-44]
  public static final int GRIPPER_MOTOR_ID = 40;

  // intake [45-49]
  public static final int INTAKE_SPINNY_MOTOR_ID = 45;
  public static final int INTAKE_DEPLOY_MOTOR_ID = 46;

  // climb [50-54]
  public static final int CLIMB_PIVOT_MOTOR_LEFT_ID = 50;
  public static final int CLIMB_PIVOT_MOTOR_RIGHT_ID = 51;
  public static final int CLIMB_PIVOT_CANCODER_ID = 52;

  // launcher [55-59]
  public static final int LAUNCHER_MOTOR_LEFT_ID = 55;
  public static final int LAUNCHER_MOTOR_RIGHT_ID = 56;
  // Dedicated hood motor (separate from flywheel motors)
  public static final int LAUNCHER_HOOD_MOTOR_ID = 57;

  // DIO sensors
  public static final int ELEVATOR_ZERO_BUTTON = 0;
  public static final int INTAKE_SENSOR = 1;
}
