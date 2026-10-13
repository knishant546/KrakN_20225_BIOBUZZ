package org.firstinspires.ftc.teamcode.AutoAppl;

/** Counts yellow arrivals once; another count requires a stable empty sensor gap. */
public final class YellowPollenCounter {
    private boolean latched;
    private double yellowSince=Double.NaN,clearSince=Double.NaN;

    public boolean update(double now, float red, float green, float blue, float alpha) {
        if(!Float.isFinite(red)||!Float.isFinite(green)||!Float.isFinite(blue)||!Float.isFinite(alpha)) {
            yellowSince=clearSince=Double.NaN; return false;
        }
        if(latched) {
            // A change from yellow to another color is NOT evidence of a new ball.
            if(alpha<HiveAutoConfig.COLOR_CLEAR_ALPHA) {
                if(Double.isNaN(clearSince))clearSince=now;
                if(now-clearSince>=HiveAutoConfig.COLOR_CLEAR_SECONDS)latched=false;
            } else clearSince=Double.NaN;
            return false;
        }
        float max=Math.max(red,Math.max(green,blue)),min=Math.min(red,Math.min(green,blue));
        float delta=max-min;
        double hue=0;
        if(delta>0) {
            if(max==red)hue=60*((green-blue)/delta);
            else if(max==green)hue=60*(2+(blue-red)/delta);
            else hue=60*(4+(red-green)/delta);
            if(hue<0)hue+=360;
        }
        boolean yellow=alpha>=HiveAutoConfig.COLOR_PRESENT_ALPHA && max>0 &&
                delta/max>=HiveAutoConfig.YELLOW_MIN_SATURATION &&
                hue>=HiveAutoConfig.YELLOW_MIN_HUE && hue<=HiveAutoConfig.YELLOW_MAX_HUE;
        if(!yellow) { yellowSince=Double.NaN; return false; }
        if(Double.isNaN(yellowSince))yellowSince=now;
        if(now-yellowSince<HiveAutoConfig.COLOR_DETECT_SECONDS)return false;
        latched=true; yellowSince=clearSince=Double.NaN; return true;
    }
}
