package org.firstinspires.ftc.teamcode.lib.drivers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@I2cDeviceType
@DeviceProperties(name = "VEML3328 Color Sensor", xmlTag = "VEML3328")
public class VEML3328 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS = new I2cAddr(0x10);
    private static final Logger log = LoggerFactory.getLogger(VEML3328.class);

    public VEML3328(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        byte[] config = {0, 0};
        deviceClient.write(0x00, config);

        return 0 == deviceClient.read8(0x00);
    }

    /**
     * Gets the 3 RGB colors from the VEML3328, and combines them into a <code>NormalizedRGBA</code>
     * @return The <code>NormalizedRGBA</code> with all 3 colors, but no alpha channel to save I2C calls
     */
    public NormalizedRGBA getColor() {
        NormalizedRGBA color = new NormalizedRGBA();
        //color.alpha = deviceClient.read8(0x94) + 256*deviceClient.read8(0x95);

        byte[] red = deviceClient.read(0x05, 2);
        byte[] green = deviceClient.read(0x06, 2);
        byte[] blue = deviceClient.read(0x07, 2);

        Log.i("raw red", "" + ((int)red[1] << 8 | red[0]));
        Log.i("raw green", "" + ((int)green[1] << 8 | green[0]));
        Log.i("raw blue", "" + ((int)blue[1] << 8 | blue[0]));

        color.red = ((int)red[1] << 8 | red[0])/65536f;
        color.green = ((int)green[1] << 8 | green[0])/65536f;
        color.blue = ((int)blue[1] << 8 | blue[0])/65536f;
        return color;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "APDS9960 Color/Proximity/Gesture Sensor";
    }

    /**
     *
     * @param hmap the current robot hardware map
     * @param name the name set in the configuration
     * @return an initialized APDS9960
     */
    public static VEML3328 fromHMap(HardwareMap hmap, String name) {
        return hmap.get(VEML3328.class, name);
    }
}
