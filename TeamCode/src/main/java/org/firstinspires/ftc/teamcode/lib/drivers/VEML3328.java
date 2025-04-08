package org.firstinspires.ftc.teamcode.lib.drivers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "VEML3328 Color Sensor", xmlTag = "VEML3328")
public class VEML3328 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS = new I2cAddr(0x10);

    public VEML3328(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    /**
     * Gets the 3 RGB colors from the APDS9960, and combines them into a <code>NormalizedRGBA</code>
     * @return The <code>NormalizedRGBA</code> with all 3 colors, but no alpha channel to save I2C calls
     */
    public NormalizedRGBA getColor() {
        NormalizedRGBA color = new NormalizedRGBA();
        //color.alpha = deviceClient.read8(0x94) + 256*deviceClient.read8(0x95);

        byte[] red = deviceClient.read(0x05);
        byte[] green = deviceClient.read(0x06);
        byte[] blue = deviceClient.read(0x07);

        color.red = (red[0] + red[1] << 0x08)/256;
        color.green = (green[0] + green[1] << 0x08)/256;
        color.blue = (blue[0] + blue[1] << 0x08)/256;
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
