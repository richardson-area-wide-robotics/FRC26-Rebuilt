# Code Structure Flowchart

```mermaid
flowchart TD
    subgraph frc_robot[frc.robot]
        BuildConstants[BuildConstants]
        CommonConstants[CommonConstants]
        Main[Main]
        Robot[Robot]
    end
    subgraph frc_robot_common_annotations[frc.robot.common.annotations]
        DashboardVariable[DashboardVariable]
        NamedAuto[NamedAuto]
        Robot[Robot]
    end
    subgraph frc_robot_common_components_dashboard[frc.robot.common.components.dashboard]
        DashboardAutoUpdater[DashboardAutoUpdater]
    end
    subgraph frc_robot_common_components_dashboard_diagnostics[frc.robot.common.components.dashboard.diagnostics]
        CANDiagnostics[CANDiagnostics]
    end
    subgraph frc_robot_common_components_diagnostics[frc.robot.common.components.diagnostics]
        ArmProfileCalibrator[ArmProfileCalibrator]
        BumpCrossingDiagnostic[BumpCrossingDiagnostic]
        CalibrationManeuvers[CalibrationManeuvers]
        CalibrationStore[CalibrationStore]
        DeployTravelCalibrator[DeployTravelCalibrator]
        DriftMonitor[DriftMonitor]
        DriveAutoCalibrator[DriveAutoCalibrator]
        DriveCharacterization[DriveCharacterization]
        DriveSysId[DriveSysId]
        ExpectationMonitor[ExpectationMonitor]
        GamePieceCounter[GamePieceCounter]
        HardStopDetector[HardStopDetector]
        LoadCalibrationRoutine[LoadCalibrationRoutine]
        LoadCalibrator[LoadCalibrator]
        ManeuverRunner[ManeuverRunner]
        MotorLoadMonitor[MotorLoadMonitor]
        RotationAccumulator[RotationAccumulator]
        RotationalInertiaCalibrator[RotationalInertiaCalibrator]
        SysIdRegression[SysIdRegression]
        TractionCalibrator[TractionCalibrator]
        TunableNumber[TunableNumber]
        ValidationSuite[ValidationSuite]
        VisionCalibration[VisionCalibration]
    end
    subgraph frc_robot_common_components[frc.robot.common.components]
        EasyBreakBeam[EasyBreakBeam]
        EasyMotor[EasyMotor]
        NamedAutoRegistry[NamedAutoRegistry]
        PathPlannerConfig[PathPlannerConfig]
        RobotContainerRegistry[RobotContainerRegistry]
        RobotExceptionHandler[RobotExceptionHandler]
        RobotUtils[RobotUtils]
        TeamUtils[TeamUtils]
    end
    subgraph frc_robot_common_components_hardware[frc.robot.common.components.hardware]
        SwerveModuleHardware[SwerveModuleHardware]
        TankHardware[TankHardware]
    end
    subgraph frc_robot_common[frc.robot.common]
        DefaultContainer[DefaultContainer]
        LocalADStarAK[LocalADStarAK]
    end
    subgraph frc_robot_common_gyro[frc.robot.common.gyro]
        RAWRNavX2[RAWRNavX2]
        RAWRQuestNav[RAWRQuestNav]
    end
    subgraph frc_robot_common_interfaces[frc.robot.common.interfaces]
        IDiagnostic[IDiagnostic]
        IMU[IMU]
        IRobotContainer[IRobotContainer]
        ModulePositionSupplier[ModulePositionSupplier]
    end
    subgraph frc_robot_common_subsystems[frc.robot.common.subsystems]
        DashboardSubsystem[DashboardSubsystem]
        SingleMotorSubsystem[SingleMotorSubsystem]
    end
    subgraph frc_robot_common_subsystems_drive[frc.robot.common.subsystems.drive]
        DriveStraightClosedLoop[DriveStraightClosedLoop]
        SwerveDriveSubsystem[SwerveDriveSubsystem]
        TankDriveSubsystem[TankDriveSubsystem]
        TurnToRelativeHeading[TurnToRelativeHeading]
    end
    subgraph frc_robot_common_subsystems_vision[frc.robot.common.subsystems.vision]
        FieldLayoutLoader[FieldLayoutLoader]
        VisionConstants[VisionConstants]
        VisionSubsystem[VisionSubsystem]
    end
    subgraph frc_robot_common_swerve[frc.robot.common.swerve]
        Configs[Configs]
        MAXSwerveModule[MAXSwerveModule]
    end
    subgraph frc_robot_rebuilt_components[frc.robot.rebuilt.components]
        FieldState[FieldState]
        HubStatus[HubStatus]
        RobotSector[RobotSector]
    end
    subgraph frc_robot_rebuilt[frc.robot.rebuilt]
        RebuiltConstants[RebuiltConstants]
        RebuiltContainer[RebuiltContainer]
        RebuiltValidation[RebuiltValidation]
    end
    subgraph frc_robot_rebuilt_states[frc.robot.rebuilt.states]
        FieldRegions[FieldRegions]
        RobotStateMachine[RobotStateMachine]
        ShooterRangeModel[ShooterRangeModel]
    end
    subgraph frc_robot_rebuilt_subsystems[frc.robot.rebuilt.subsystems]
        Feeder[Feeder]
        Intake[Intake]
        JamClearing[JamClearing]
        Shooter[Shooter]
    end
    subgraph frc_robot_rebuilt_subsystems_smart[frc.robot.rebuilt.subsystems.smart]
        DynamicPather[DynamicPather]
        MathShoot[MathShoot]
        RobotSectorEvaluator[RobotSectorEvaluator]
        ScoringLocationLookup[ScoringLocationLookup]
    end
    BaseInstanceable_CANDiagnostics[BaseInstanceable<CANDiagnostics]
    CANDiagnostics -->|extends| BaseInstanceable_CANDiagnostics
    LoggableHardware[LoggableHardware]
    RAWRNavX2 -->|extends| LoggableHardware
    AutoCloseable[AutoCloseable]
    IMU -->|extends| AutoCloseable
    SubsystemBase[SubsystemBase]
    DashboardSubsystem -->|extends| SubsystemBase
    Command[Command]
    DriveStraightClosedLoop -->|extends| Command
    SwerveDriveSubsystem -->|extends| SubsystemBase
    TankDriveSubsystem -->|extends| SubsystemBase
    TurnToRelativeHeading -->|extends| Command
    SingleMotorSubsystem -->|extends| DashboardSubsystem
    VisionSubsystem -->|extends| SubsystemBase
    Feeder -->|extends| DashboardSubsystem
    Intake -->|extends| DashboardSubsystem
    Shooter -->|extends| DashboardSubsystem
    RobotSectorEvaluator -->|extends| DashboardSubsystem
    LoggedRobot[LoggedRobot]
    Robot -->|extends| LoggedRobot
    CANDiagnostics -.implements.-> IDiagnostic
    Thread_UncaughtExceptionHandler[Thread.UncaughtExceptionHandler]
    RobotExceptionHandler -.implements.-> Thread_UncaughtExceptionHandler
    DefaultContainer -.implements.-> IRobotContainer
    RAWRNavX2 -.implements.-> IMU
    org_lasarobotics_hardware_IMU[org.lasarobotics.hardware.IMU]
    RAWRNavX2 -.implements.-> org_lasarobotics_hardware_IMU
    RAWRQuestNav -.implements.-> IMU
    Pathfinder[Pathfinder]
    LocalADStarAK -.implements.-> Pathfinder
    SwerveDriveSubsystem -.implements.-> ModulePositionSupplier
    TankDriveSubsystem -.implements.-> AutoCloseable
    RebuiltContainer -.implements.-> IRobotContainer
    ArmProfileCalibrator --> Intake
    BumpCrossingDiagnostic --> SwerveDriveSubsystem
    CalibrationStore --> CalibrationStore
    DeployTravelCalibrator --> Intake
    DriveAutoCalibrator --> SwerveDriveSubsystem
    DriveAutoCalibrator --> VisionSubsystem
    DriveSysId --> SwerveDriveSubsystem
    ExpectationMonitor --> ExpectationMonitor
    GamePieceCounter --> MotorLoadMonitor
    LoadCalibrationRoutine --> Intake
    LoadCalibrationRoutine --> Feeder
    LoadCalibrationRoutine --> Shooter
    ManeuverRunner --> SwerveDriveSubsystem
    RotationalInertiaCalibrator --> SwerveDriveSubsystem
    TractionCalibrator --> SwerveDriveSubsystem
    DriveStraightClosedLoop --> SwerveDriveSubsystem
    TankDriveSubsystem --> TankHardware
    TurnToRelativeHeading --> SwerveDriveSubsystem
    RobotSectorEvaluator --> SwerveDriveSubsystem
    Robot --> IRobotContainer
    style BuildConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DashboardVariable fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style NamedAuto fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style Robot fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style DashboardAutoUpdater fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style CANDiagnostics fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ArmProfileCalibrator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style BumpCrossingDiagnostic fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style CalibrationManeuvers fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style CalibrationStore fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DeployTravelCalibrator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DriftMonitor fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DriveAutoCalibrator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DriveCharacterization fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DriveSysId fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ExpectationMonitor fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style GamePieceCounter fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style HardStopDetector fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style LoadCalibrationRoutine fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style LoadCalibrator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ManeuverRunner fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style MotorLoadMonitor fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RotationAccumulator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RotationalInertiaCalibrator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SysIdRegression fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TractionCalibrator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TunableNumber fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ValidationSuite fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style VisionCalibration fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style EasyBreakBeam fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style EasyMotor fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SwerveModuleHardware fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style TankHardware fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style NamedAutoRegistry fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style PathPlannerConfig fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotContainerRegistry fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotExceptionHandler fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotUtils fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TeamUtils fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DefaultContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RAWRNavX2 fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RAWRQuestNav fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style IDiagnostic fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style IMU fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style IRobotContainer fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style ModulePositionSupplier fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style LocalADStarAK fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DashboardSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DriveStraightClosedLoop fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SwerveDriveSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TankDriveSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TurnToRelativeHeading fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SingleMotorSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style FieldLayoutLoader fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style VisionConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style VisionSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Configs fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style MAXSwerveModule fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style CommonConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Main fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style FieldState fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style HubStatus fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotSector fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RebuiltConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RebuiltContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RebuiltValidation fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style FieldRegions fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotStateMachine fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ShooterRangeModel fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Feeder fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Intake fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style JamClearing fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Shooter fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DynamicPather fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style MathShoot fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotSectorEvaluator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ScoringLocationLookup fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Robot fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style BaseInstanceable_CANDiagnostics fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style LoggableHardware fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style AutoCloseable fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style SubsystemBase fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style Command fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style LoggedRobot fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style Thread_UncaughtExceptionHandler fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style org_lasarobotics_hardware_IMU fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style Pathfinder fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
```
