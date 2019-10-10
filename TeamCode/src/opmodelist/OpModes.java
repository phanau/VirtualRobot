package opmodelist;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;

/**
 * Container class for registering OpModes.
 */
public class OpModes {
    /**
     * The list of OpModes that are to be displayed in the OpMode dropdown list.
     *
     * Add the names of your OpModes to this list.
     */
    public static final ObservableList<String> opModes =
            FXCollections.observableArrayList(

                    "TwoWheelDemo", "TwoWheelArcadeDrive", "MechBotDemo", "MechBotAutoDemo", "MyTankDrive", "PosIntDriveTestOp",
                    "AbsoluteGyroDrive1", "AbsoluteSquirrelyGyroDrive1", "SquirrelyDrive1", "SquirrelyDrive2", "TankDrive1",
                    "TankDrivePosInt", "SquirrelyGyroDriveTestOp", "SquirrelyPosIntDriveTestOp", "SkystoneAutoBlue1"

            );
}
