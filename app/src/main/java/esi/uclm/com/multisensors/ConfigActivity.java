package esi.uclm.com.multisensors;

/**
 * Created by Javier on 27/10/2014.
 */

import android.os.Bundle;
import android.preference.Preference;
import android.preference.Preference.OnPreferenceChangeListener;
import android.preference.PreferenceActivity;
import android.preference.PreferenceFragment;

public class ConfigActivity extends PreferenceActivity implements OnPreferenceChangeListener{

    public static final String FUSION_PREFERENCE = "fusion_preference";

    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

		/*
		 * Read preferences resources available at res/xml/preferences.xml
		 */
        addPreferencesFromResource(R.xml.preferences);
    }

    @Override
    public boolean onPreferenceChange(Preference preference, Object newValue)
    {
        return false;
    }

}
