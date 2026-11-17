package org.firstinspires.ftc.teamcode.timber_util;


import android.util.Pair;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Adjuster {

    private HashMap<String, Double> map = new HashMap<>();
    private BooleanSupplier incr, decr, mult, div;
    private String currentKey;

    public Adjuster(BooleanSupplier incr, BooleanSupplier decr, BooleanSupplier mult,
                    BooleanSupplier div, Pair<String,Double>... pairs){
        this.incr = incr;
        this.decr = decr;
        this.mult = mult;
        this.div = div;
        currentKey = null;
        for (Pair<String,Double> p: pairs){
            map.put(p.first, p.second);
            if (currentKey == null) currentKey = p.first;
        }
    }

    public String getCurrentKey(){
        return currentKey;
    }

    public void setCurrentKey(String key){
        currentKey = key;
    }

    public double getCurrentValue(){
        return map.get(currentKey);
    }

    public double getValue(String key){
        return map.get(key);
    }

    public void update(){
        boolean increment = incr.getAsBoolean();
        boolean decrement = decr.getAsBoolean();
        boolean multiply = mult.getAsBoolean();
        boolean divide = div.getAsBoolean();

        double val = map.get(currentKey);
        int log = (int)Math.floor(Math.log10(val));
        double incr_decr = Math.pow(10.0, log - 1);

        if (increment){
            val += incr_decr;
        } else if (decrement){
            val -= incr_decr;
        } else if (multiply){
            val *= 10;
        } else if (divide){
            val /= 10;
        }

        map.replace(currentKey, val);

    }

    public String toString(){
        StringBuilder sb = new StringBuilder();
        for (String key: map.keySet()){
            sb.append("\n" + key + ": " + map.get(key));
        }
        return sb.toString();
    }

}
