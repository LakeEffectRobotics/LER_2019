//  OPB732WZ Reflective Optical Sensor
//  This thing will switch from LOW to HIGH at 1.3 V. 

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class TapeSensor extends DigitalInput {
    public TapeSensor(int channel) {
        super(channel);
    }

    //  this.get() will return false on tape and true otherwise because the voltage going to the RoboRIO is inverted in Jeremy's calibration circuit
    //  this.isOnTape() inverts this.get() and will return true on tape and false otherwise
    public boolean isOnTape() {
        return !this.get();
    }
}
