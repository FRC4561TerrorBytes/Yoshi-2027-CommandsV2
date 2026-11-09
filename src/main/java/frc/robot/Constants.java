// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.wpilib.framework.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying FROM a log file. */
    REPLAY
  }

  public static final int ELEVATORMOTOR1_ID = 0;
  public static final int ELEVATORMOTOR2_ID = 0;
  public static final int ELEVATORCANCODER1_ID = 0;
  public static final int ELEVATORCANCODER1_TEETH = 18;
  public static final int ELEVATORCANCODER2_ID = 0;
  public static final int ELEVATORCANCODER2_TEETH = 19;  
  public static final double ELEVATOR_GEAR_RATIO = 20/40;
  public static final double ELEVATOR_ROTATIONS_PER_INCH = 1; // edit and find out actual
  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 60;
  public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 0;
  public static final int ELEVATORCANCODER1_KEY = 19;
  public static final int ELEVATORCANCODER2_KEY = 324;
  public static final int ELEVATOR_MAX_STEPS = 342;

  //in inches
  public static final double ELEVATOR_MAX_RANGE = 64.9375;
}
