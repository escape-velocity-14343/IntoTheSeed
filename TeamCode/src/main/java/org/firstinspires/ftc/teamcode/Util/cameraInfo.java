package org.firstinspires.ftc.teamcode.Util;

public class cameraInfo {
    String name;
    // In inches
    double xoffset = 0;
    double yoffset = 0;
    double zoffset = 0;

    double resolutionHeight = 0;
    double resolutionWidth = 0;

    public cameraInfo(String name, double x, double y, double z, double resolutionWidth, double resolutionHeight){
        this.name = name;
        this.xoffset = x;
        this.yoffset = y;
        this.zoffset = z;
        this.resolutionHeight = resolutionHeight;
        this.resolutionWidth = resolutionWidth;
    }

    public String name(){
        return name;
    }

    public double x(){
        return xoffset;
    }

    public double y(){
        return yoffset;
    }

    public double z(){
        return zoffset;
    }

    /**
     * Width index 0 x Height index 1
     */
    public Double[] getResolution(){
        return new Double[]{resolutionWidth, resolutionHeight};
    }
}
