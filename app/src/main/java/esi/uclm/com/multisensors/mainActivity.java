package esi.uclm.com.multisensors;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.TextView;

import java.text.DecimalFormat;

import static android.util.FloatMath.*;

public class mainActivity extends Activity implements SensorEventListener{

    private static final float NS2S = 1.0f / 1000000000.0f;
    private static final float EPSILON = 0.000000001f;
    private static final int MEAN_FILTER_WINDOW = 10;
    private static final int MIN_SAMPLE_COUNT = 30;


    private DecimalFormat df;

    private float[] deltaRotationVector;
    private float[] deltaRotationMatrix;

    private float[] gyroscopeOrientation;
    private float[] currentRotationMatrix;

    // accelerometer and magnetometer based rotation matrix
    private float[] initialRotationMatrix;

    // accelerometer vector
    private float[] acceleration;

    // magnetic field vector
    private float[] magnetic;

    private MeanFilter accelerationFilter;
    private MeanFilter magneticFilter;

    // TIEMPOS DE REFRESCO
    private float timestampOld = 0;

    private int accelerationSampleCount;
    private int magneticSampleCount;

    //COMPROBACIONES
    boolean hasInitialOrientation = false;
    boolean stateInitializedCalibrated = false;

    private SensorManager mSensorManager;

    TextView calibrationX;
    TextView calibrationY;
    TextView calibrationZ;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initUI();
        initMaths();
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        initFilters();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
        //TODO
    }

    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            onAccelerationSensorChanged(event.values, event.timestamp);
        }

        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            onMagneticSensorChanged(event.values, event.timestamp);
        }

        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            onGyroscopeSensorChanged(event.values, event.timestamp);
        }
    }

    public void onAccelerationSensorChanged(float[] acceleration, long timeStamp) {
        // Get a local copy of the raw magnetic values from the device sensor.
        System.arraycopy(acceleration, 0, this.acceleration, 0,
                acceleration.length);

        // Use a mean filter to smooth the sensor inputs
        this.acceleration = accelerationFilter.filterFloat(this.acceleration);

        // Count the number of samples received.
        accelerationSampleCount++;

        // Only determine the initial orientation after the acceleration sensor
        // and magnetic sensor have had enough time to be smoothed by the mean
        // filters. Also, only do this if the orientation hasn't already been
        // determined since we only need it once.
        if (accelerationSampleCount > MIN_SAMPLE_COUNT
                && magneticSampleCount > MIN_SAMPLE_COUNT
                && !hasInitialOrientation)
        {
            calculateOrientation();
        }
    }

    public void onMagneticSensorChanged(float[] magnetic, long timeStamp) {
        // Get a local copy of the raw magnetic values from the device sensor.
        System.arraycopy(magnetic, 0, this.magnetic, 0, magnetic.length);

        // Use a mean filter to smooth the sensor inputs
        this.magnetic = magneticFilter.filterFloat(this.magnetic);

        // Count the number of samples received.
        magneticSampleCount++;
    }

    public void onGyroscopeSensorChanged(float[] gyroscope, long timestamp) {

        // don't start until first accelerometer/magnetometer orientation has
        // been acquired
        if (!hasInitialOrientation)
        {
            return;
        }

        // Initialization of the gyroscope based rotation matrix
        if (!stateInitializedCalibrated) {
            currentRotationMatrix = matrixMultiplication(
                    currentRotationMatrix, initialRotationMatrix);

            stateInitializedCalibrated = true;
        }
        // This timestep's delta rotation to be multiplied by the current rotation
        // after computing it from the gyro sample data.
        if (timestamp != 0) {
            final float dT = (timestamp - timestampOld) * NS2S;
            // Axis of the rotation sample, not normalized yet.
            float axisX = gyroscope[0];
            float axisY = gyroscope[1];
            float axisZ = gyroscope[2];

            // Calculate the angular speed of the sample
            float omegaMagnitude = sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

            // Normalize the rotation vector if it's big enough to get the axis
            // (that is, EPSILON should represent your maximum allowable margin of error)
            if (omegaMagnitude > EPSILON) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }

            // Integrate around this axis with the angular speed by the timestep
            // in order to get a delta rotation from this sample over the timestep
            // We will convert this axis-angle representation of the delta rotation
            // into a quaternion before turning it into the rotation matrix.
            float thetaOverTwo = omegaMagnitude * dT / 2.0f;

            float sinThetaOverTwo = sin(thetaOverTwo);
            float cosThetaOverTwo = cos(thetaOverTwo);

            deltaRotationVector[0] = sinThetaOverTwo * axisX;
            deltaRotationVector[1] = sinThetaOverTwo * axisY;
            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
            deltaRotationVector[3] = cosThetaOverTwo;

            SensorManager.getRotationMatrixFromVector(
                    deltaRotationMatrix,
                    deltaRotationVector);

            currentRotationMatrix = matrixMultiplication(
                    currentRotationMatrix,
                    deltaRotationMatrix);

            SensorManager.getOrientation(currentRotationMatrix,
                    gyroscopeOrientation);
        }
        timestampOld = timestamp;

        // User code should concatenate the delta rotation we computed with the current rotation
        // in order to get the updated rotation.
        // rotationCurrent = rotationCurrent * deltaRotationMatrix;

        calibrationX.setText(df.format(Math
                .toDegrees(gyroscopeOrientation[0])));
        calibrationY.setText(df.format(Math
                .toDegrees(gyroscopeOrientation[1])));
        calibrationZ.setText(df.format(Math
                .toDegrees(gyroscopeOrientation[2])));
    }

    private float[] matrixMultiplication(float[] a, float[] b) {
        float[] result = new float[9];

        result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
        result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
        result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

        result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
        result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
        result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

        result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
        result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
        result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

        return result;
    }

    private void initMaths(){

        acceleration = new float[3];
        magnetic = new float[3];

        initialRotationMatrix = new float[9];

        deltaRotationVector = new float[4];
        deltaRotationMatrix = new float[9];
        currentRotationMatrix = new float[9];
        gyroscopeOrientation = new float[3];

        // Initialize the current rotation matrix as an identity matrix...
        currentRotationMatrix[0] = 1.0f;
        currentRotationMatrix[4] = 1.0f;
        currentRotationMatrix[8] = 1.0f;
    }

    private void initUI(){
        // Get a decimal formatter for the text views
        df = new DecimalFormat("#.##");

        // Initialize the calibrated text views
        calibrationX = (TextView) this
                .findViewById(R.id.calibrationX);
        calibrationY = (TextView) this
                .findViewById(R.id.calibrationY);
        calibrationZ = (TextView) this
                .findViewById(R.id.calibrationZ);
    }

    private void initFilters() {
        accelerationFilter = new MeanFilter();
        accelerationFilter.setWindowSize(MEAN_FILTER_WINDOW);

        magneticFilter = new MeanFilter();
        magneticFilter.setWindowSize(MEAN_FILTER_WINDOW);
    }

    private void reset(){

        mSensorManager.unregisterListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));

        mSensorManager.unregisterListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));

        initMaths();

        accelerationSampleCount = 0;
        magneticSampleCount = 0;

        hasInitialOrientation = false;
        stateInitializedCalibrated = false;
    }

    private void restart(){

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);

    }

    private void calculateOrientation() {
        hasInitialOrientation = SensorManager.getRotationMatrix(
                initialRotationMatrix, null, acceleration, magnetic);

        // Remove the sensor observers since they are no longer required.
        if (hasInitialOrientation)
        {
            mSensorManager.unregisterListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));
            mSensorManager.unregisterListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));
        }
    }


    /* ESTADOS DE LA APLICACION */
    @Override
    public void onResume(){
        super.onResume();

        restart();
    }

    @Override
    public void onPause(){
        super.onPause();

        reset();
    }
}