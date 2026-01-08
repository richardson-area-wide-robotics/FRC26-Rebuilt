package frc.robot.common.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.NamedAutoRegistry;
import frc.robot.common.components.dashboard.DashboardAutoUpdater;

/**
 * A SubsystemBase that automatically registers its instance
 * for @DashboardVariable and @NamedAuto tracking.
 * <br>
 * All subsystems should extend from this
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public abstract class DashboardSubsystem extends SubsystemBase {

    public DashboardSubsystem() {
        super();
        NamedAutoRegistry.register(this);
        DashboardAutoUpdater.register(this);
    }

    public DashboardSubsystem(String name) {
        super(name);
        NamedAutoRegistry.register(this);
        DashboardAutoUpdater.register(this);
    }
}
