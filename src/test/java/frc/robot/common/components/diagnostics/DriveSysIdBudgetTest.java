package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Guards that the SysId characterisation still fits on the carpet the team actually has.
 *
 * <p>This exists because the failure mode is a robot driving into a wall, and the tempting change
 * that causes it looks entirely reasonable: lengthen the ramp to get more voltage range and a better
 * fit. The stock SysId ramp of 1 V/s for 6 s covers 8.10 m on this drivetrain against 8.53 m of
 * carpet, so the out-of-the-box configuration is already past the limit — it is not a hypothetical.
 *
 * <p>These are arithmetic checks on the planned distance, not a substitute for the runtime distance
 * abort. The prediction uses the nominal kV, and the measured kV is the thing SysId exists to
 * produce, so the prediction is circular by nature. Both guards are needed: this one catches a bad
 * configuration before anyone drives, the abort catches the prediction being wrong.
 */
class DriveSysIdBudgetTest {

    @Test
    @DisplayName("Half a field is 28 ft, and the reserve leaves usable runway")
    void carpetBudgetIsSane() {
        assertEquals(8.5344, DriveSysId.HALF_FIELD_METERS, 1e-4, "28 ft in metres");

        double usable = DriveSysId.HALF_FIELD_METERS - DriveSysId.RESERVED_METERS;
        assertTrue(usable > 6.0,
                "after reserving robot length and stopping distance there must still be runway; "
                        + "usable = " + usable);
    }

    @Test
    @DisplayName("The distance abort sits inside the usable carpet")
    void abortFitsTheCarpet() {
        double usable = DriveSysId.HALF_FIELD_METERS - DriveSysId.RESERVED_METERS;

        assertTrue(DriveSysId.MAX_RUN_METERS <= usable,
                "the abort limit " + DriveSysId.MAX_RUN_METERS + " m must be within the usable "
                        + usable + " m, or the abort fires only after the robot has hit something");
    }

    @Test
    @DisplayName("The planned quasistatic ramp fits inside the abort limit")
    void plannedRampFitsInsideTheAbort() {
        double planned = DriveSysId.plannedRampMeters();

        assertTrue(planned < DriveSysId.MAX_RUN_METERS,
                "the ramp is planned to cover " + planned + " m but aborts at "
                        + DriveSysId.MAX_RUN_METERS + " m — every run would be cut short, which is "
                        + "not a safety net, it is a broken configuration");

        // And with real margin, not just barely. A run that only just fits leaves nothing for a
        // drivetrain that turns out faster than nominal.
        assertTrue(planned < DriveSysId.MAX_RUN_METERS * 0.85,
                "planned " + planned + " m is within 15% of the abort limit; too tight");
    }

    @Test
    @DisplayName("The planned ramp fits the carpet even with nothing reserved")
    void plannedRampFitsRawCarpet() {
        assertTrue(DriveSysId.plannedRampMeters() < DriveSysId.HALF_FIELD_METERS,
                "sanity floor: the plan must not exceed the carpet itself");
    }

    @Test
    @DisplayName("The ramp still reaches enough voltage to fit kV against")
    void rampReachesUsefulVoltage() {
        // The space saving is only worth having if the fit still has range to work with. Below about
        // 4 V the velocity span is too small to separate kS from kV reliably.
        double volts = DriveSysId.rampFinalVolts();

        assertTrue(volts >= 4.0,
                "ramp reaches only " + volts + " V — too little range to separate kS from kV");
        assertTrue(volts <= 8.0,
                "ramp reaches " + volts + " V, which is more range than the space allows; distance "
                        + "grows with the square of the ramp time");
    }

    @Test
    @DisplayName("Nominal kV matches the corrected Vortex free speed")
    void nominalKvUsesTheCorrectedFreeSpeed() {
        // Ties the distance prediction to the drive constants, so changing the wheel or gearing
        // re-prices the runway automatically. 12 V / 6.015 m/s is about 1.995.
        //
        // Was 2.09, which came from a 4.714:1 reduction — the template's 22T spur rather than this
        // robot's 21T Extra High 1. The runway predictions all scale with kV, so a wrong reduction
        // silently mis-prices how far each SysId run travels.
        assertEquals(1.995, DriveSysId.nominalKv(), 0.02,
                "if this drifted, either the drive constants changed or one of the three template "
                        + "defaults is back: motor free speed, spur gear, or module spacing");
    }
}
