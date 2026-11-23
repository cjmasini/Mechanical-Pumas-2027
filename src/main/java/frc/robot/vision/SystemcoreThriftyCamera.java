package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * VisionCamera subclass for a ThriftyCam (or any UVC USB camera) plugged directly
 * into one of the 2027 Systemcore's USB ports.
 *
 * The 2027 Systemcore includes built-in Limelight Vision software capable of running
 * up to 4 USB cameras natively on NetworkTables under {@code limelightsc0} through
 * {@code limelightsc3}. Because it uses the Limelight NetworkTables protocol, extending
 * {@link LimelightCamera} reuses all of our MegaTag 2 orientation feeding, spin-rate
 * rejection, field-bounds checking, and distance-scaled standard deviations right now.
 */
public class SystemcoreThriftyCamera extends LimelightCamera {

    /**
     * Default NetworkTables camera names assigned by Systemcore's built-in vision stack
     * for cameras plugged directly into its USB ports.
     */
    public enum SystemcoreUsbPort {
        USB0("limelightsc0"),
        USB1("limelightsc1"),
        USB2("limelightsc2"),
        USB3("limelightsc3");

        private final String tableName;

        SystemcoreUsbPort(String tableName) {
            this.tableName = tableName;
        }

        /**
         * Returns the NetworkTables table name used for this Systemcore USB port.
         */
        public String getTableName() {
            return tableName;
        }
    }

    // WPILib 2027 LimelightLib reference (once WPILib 2027 + 2027 LimelightLib release at Kickoff,
    // you can optionally inspect or switch to CTRE/WPILib's built-in Limelight constants):
    // Limelight camera = new Limelight(Limelight.SYSTEMCORE_USB0); // maps to "limelightsc0"

    /**
     * Creates a Systemcore-hosted ThriftyCam using one of the standard Systemcore USB ports.
     *
     * @param usbPort       Which Systemcore USB port (USB0..USB3) the camera is plugged into
     * @param robotToCamera Physical 3D transform from robot center to camera lens
     */
    public SystemcoreThriftyCamera(SystemcoreUsbPort usbPort, Transform3d robotToCamera) {
        super(usbPort.getTableName(), robotToCamera);
    }

    /**
     * Creates a Systemcore-hosted ThriftyCam with a custom NetworkTables name
     * (if renamed in the Systemcore vision web UI).
     *
     * @param customTableName Custom NetworkTables table name configured on Systemcore
     * @param robotToCamera   Physical 3D transform from robot center to camera lens
     */
    public SystemcoreThriftyCamera(String customTableName, Transform3d robotToCamera) {
        super(customTableName, robotToCamera);
    }
}
