package localhost.helloworld_hw12;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    SeekBar ctrlBar;
    TextView ctrlText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        ctrlBar = (SeekBar) findViewById(R.id.ctrlSeek1);

        ctrlText = (TextView) findViewById(R.id.ctrlText1);
        ctrlText.setText("Lorem ipsum.");

        setControlListener();
    }

    protected void setControlListener() {
        ctrlBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
                ctrlText.setText("The value is "+progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
}
