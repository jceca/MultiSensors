package esi.uclm.com.multisensors;

/**
 * Created by Javier on 27/10/2014.
 */
public interface FusedGyroscopeSensorListener {
    public void onAngularVelocitySensorChanged(float[] angularVelocity, long timeStamp);
}
