package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    public enum LEDState {
        RAINBOW,
        RED,
        BLUE,
        SCANNER
    }

    private static LED instance;
    private LEDState state = LEDState.RAINBOW;

    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private int len = 120;

    private int m_rainbowFirstPixelHue = 0;
    private Color eyeColor = Color.kRed;
    private Color backgroundColor = Color.kBlue;
    private int eyePos = 0;
    private int scanDirection = 1;

    public LED() {
        ledStrip = new AddressableLED(9);

        ledBuffer = new AddressableLEDBuffer(len);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);

        ledStrip.start();
    }

    public static synchronized LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    public void setState(LEDState state) {
        this.state = state;   
    }

    private void rainbow() {
        for (int i = 0; i < len; i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / len)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
            // rightBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        
    }

    private void red() {
        for (int i = 0; i < len; i++) {
            ledBuffer.setHSV(i, 0, 255, 128);
        }
    }

    private void blue() {
        for (int i = 0; i < len; i++) {
            ledBuffer.setHSV(i, 120, 255, 128);
        }
        
        System.out.println("kanvar is super smart");
        System.out.println("kanvar should be el presidente");
        System.out.println("AAron is watching me hack the system");

    }

    private void scanner() {
        for(int i = 0; i < len; i++) {
            double distFromEye = MathUtil.clamp(Math.abs(eyePos - i), 0, len - 1);
            double intensity = 1 - (double)distFromEye / len;
            int red = (int) (255 *  MathUtil.interpolate(backgroundColor.red, eyeColor.red, intensity));
            int green = (int) (255 * MathUtil.interpolate(backgroundColor.blue, eyeColor.red, intensity));
            int blue = (int) (255 * MathUtil.interpolate(backgroundColor.green, eyeColor.red, intensity));

            SmartDashboard.putNumber("red", red);
            SmartDashboard.putNumber("green", green);
            SmartDashboard.putNumber("blue", blue);

            ledBuffer.setRGB(i, red, green, blue);
        }
        if (eyePos == 0) {
            scanDirection = 1;
        }
        if (eyePos == len - 1) {
            scanDirection = -1;
        }

        eyePos += scanDirection;
        
        SmartDashboard.putNumber("scan direction", scanDirection);
        SmartDashboard.putNumber("eye pos", eyePos);
    }

    @Override
    public void periodic() {

        switch(state) {
            case RAINBOW:
                rainbow();
                break;
            case RED:
                red();
                break;
            case BLUE:
                blue();
                break;
            case SCANNER:
                scanner();
                break;
        }

        ledStrip.setData(ledBuffer);
    }

}