package frc.robot.common.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.testutil.HalFixture;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Pins down what units REVLib's simulation API actually uses, by measurement.
 *
 * <p>Kept as a record of findings for whoever next attempts drivetrain physics simulation. An
 * attempt during this session produced velocities 49x, then 10x, then roughly half the correct
 * value, because the unit contract was being inferred from method names rather than checked. The
 * attempt was reverted; these facts were the useful part of it.
 *
 * <p><b>What was established:</b>
 * <ul>
 *   <li>The encoder sim setters take <em>converted</em> units — whatever the conversion factors on
 *       the controller produce. {@code setVelocity(1.0)} on a drive module configured in metres per
 *       second reads back as exactly 1.0 m/s, not as RPM.</li>
 *   <li>{@code SparkSim.iterate(velocity, vbus, dt)} takes velocity in those same converted units,
 *       <em>not</em> motor RPM, despite RPM being the natural reading of the name. Position advances
 *       by exactly {@code velocity * dt}.</li>
 *   <li>Velocity readback is filtered, so it lags the value passed to {@code iterate} over several
 *       calls rather than matching immediately. Anything asserting on velocity after a single
 *       iteration will see roughly 40% of the input.</li>
 * </ul>
 *
 * <p>These tests exist so the next attempt starts from measured fact. They exercise a throwaway
 * controller on an unused CAN ID and touch no robot code.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class RevSimUnitContractTest {

  /** A CAN ID claimed by neither the robot nor any other test. */
  private static final int PROBE_CAN_ID = 60;

  /** Metres of wheel travel per motor revolution, as the drive modules are configured. */
  private static final double DRIVING_FACTOR =
      ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;

  private SparkFlex spark;
  private SparkFlexSim sim;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();

    spark = new SparkFlex(PROBE_CAN_ID, MotorType.kBrushless);

    // Configured exactly as a real drive module: position in metres, velocity in m/s.
    SparkFlexConfig config = new SparkFlexConfig();
    config.encoder
        .positionConversionFactor(DRIVING_FACTOR)
        .velocityConversionFactor(DRIVING_FACTOR / 60.0);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    sim = new SparkFlexSim(spark, DCMotor.getNeoVortex(1));
  }

  @Test
  @DisplayName("Conversion factors round-trip, but only to float precision")
  void conversionFactorsRoundTripAtFloatPrecision() {
    // Measured: the sim returns 0.0507795624434948 for a configured 0.05077956125529683 — the
    // value has been through a float somewhere in REVLib. Roughly 1.2e-9 of error, which is
    // irrelevant physically but will break an exact comparison.
    assertEquals(DRIVING_FACTOR,
        sim.getRelativeEncoderSim().getPositionConversionFactor(), 1e-7);
    assertEquals(DRIVING_FACTOR / 60.0,
        sim.getRelativeEncoderSim().getVelocityConversionFactor(), 1e-7);
  }

  @Test
  @DisplayName("Encoder sim setters take converted units, not motor RPM")
  void settersTakeConvertedUnits() {
    // If these took RPM, a set of 1.0 would read back as 1.0 * velocityConversionFactor, which
    // is about 0.00085 rather than 1.0.
    for (double value : new double[] {1.0, 100.0, 1000.0}) {
      sim.getRelativeEncoderSim().setVelocity(value);
      assertEquals(value, spark.getEncoder().getVelocity(), 1e-6,
          "setVelocity should round-trip in converted units (m/s here)");
    }

    for (double value : new double[] {1.0, 10.0}) {
      sim.getRelativeEncoderSim().setPosition(value);
      assertEquals(value, spark.getEncoder().getPosition(), 1e-6,
          "setPosition should round-trip in converted units (metres here)");
    }
  }

  @Test
  @DisplayName("iterate() integrates position as velocity times dt, in converted units")
  void iterateIntegratesInConvertedUnits() {
    // This is the fact that matters, and the one that was got wrong three times. A physics model
    // must pass linear wheel speed in m/s here — not the motor RPM that DCMotorSim reports.
    double dt = 0.020;

    for (double velocity : new double[] {1.0, 100.0, 1439.0}) {
      sim.setPosition(0);
      sim.iterate(velocity, 12.0, dt);

      assertEquals(velocity * dt, spark.getEncoder().getPosition(), 1e-4,
          "Position must advance by exactly velocity * dt, which pins the unit of the velocity "
              + "argument to the encoder's converted unit");
    }
  }

  @Test
  @DisplayName("Velocity readback is stateful — only trustworthy once a steady input has settled")
  void velocityReadbackNeedsSettling() {
    // Deliberately narrow, because the transient is not characterised. Observed values after a
    // single iterate(100) were 30.6 in one context and 635.6 in another, depending on prior
    // filter state — so a model must not read velocity back immediately and expect the input.
    //
    // What IS reproducible, and all that is claimed here: held steady, the reading converges on
    // the commanded value.
    sim.setPosition(0);
    for (int i = 0; i < 100; i++) {
      sim.iterate(100.0, 12.0, 0.020);
    }

    assertEquals(100.0, spark.getEncoder().getVelocity(), 5.0,
        "A sustained input should converge close to the commanded velocity");
  }

  @Test
  @DisplayName("Position integration is exact regardless of the velocity filter's state")
  void positionIsUnaffectedByFilterState() {
    // This is why a physics model should trust position rather than velocity readback: position
    // is a clean integral, whatever the filter is doing.
    sim.setPosition(0);
    for (int i = 0; i < 50; i++) {
      sim.iterate(2.0, 12.0, 0.020);
    }

    assertEquals(2.0 * 0.020 * 50, spark.getEncoder().getPosition(), 1e-3,
        "50 iterations at 2 m/s over 20 ms each is exactly 2 m");
  }

  @Test
  @DisplayName("A wheel speed at the physical maximum corresponds to the expected motor RPM")
  void physicalMaximumIsConsistent() {
    // Sanity arithmetic for whoever builds the physics model: at the NEO Vortex free speed of
    // 6784 RPM through the 4.50:1 Extra High 1 reduction, the wheel turns at 1508 RPM, which on a
    // 3 inch wheel is 6.015 m/s. That is the number a correct model should approach at full
    // throttle, and it matches kDriveWheelFreeSpeedRps.
    //
    // This read 5.74 m/s, from the 4.714:1 ratio the template assumes. Left as a derived assertion
    // against kDriveWheelFreeSpeedRps below so that it cannot go stale again independently.
    double motorRevsPerSecond = 6784 / 60.0;
    double wheelMetersPerSecond = motorRevsPerSecond * DRIVING_FACTOR;

    assertEquals(6.015, wheelMetersPerSecond, 0.01);
    assertEquals(ModuleConstants.kDriveWheelFreeSpeedRps, wheelMetersPerSecond, 1e-6,
        "The constant and this arithmetic must agree, or the feedforward is wrong");
  }
}
