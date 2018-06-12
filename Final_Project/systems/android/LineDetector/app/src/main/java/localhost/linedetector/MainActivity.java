package localhost.linedetector;

// import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.os.Bundle;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;

import java.io.IOException;

import android.hardware.usb.UsbManager;
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;
import android.hardware.usb.UsbDeviceConnection;

import java.io.UnsupportedEncodingException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;

import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;


public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {

    private UsbManager manager;
    private UsbSerialPort sPort;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;

    private Camera camera;
    private TextureView camTextureView;
    private SurfaceView camSurfaceView;
    private SurfaceHolder camSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView camTextView;
    private SeekBar camRedThreshold;
    private SeekBar camRedAlpha;

    static long prevtime = 0;                                                   // for FPS calculation
    static int redThreshold = 0;
    static int redAlpha = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);   // keeps the screen from turning off

        // CAMERA
        camTextView = (TextView) findViewById(R.id.cameraStatus);
        camRedThreshold = (SeekBar) findViewById(R.id.rThresh);
        camRedAlpha = (SeekBar) findViewById(R.id.rAlpha);

        // see if the app has permission to use the camera
        // ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            camSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            camSurfaceHolder = camSurfaceView.getHolder();

            camTextureView = (TextureView) findViewById(R.id.textureview);
            camTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            camTextView.setText("Started camera.");
        }
        else {
            camTextView.setText("No camera permissions.");
        }

        setRedThresholdListener();
        setRedAlphaListener();

        manager = (UsbManager) getSystemService(Context.USB_SERVICE);
    }

    private void setRedThresholdListener() {
        camRedThreshold.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                redThreshold = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    private void setRedAlphaListener() {
        camRedAlpha.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                redAlpha = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        camera = Camera.open();
        Camera.Parameters parameters = camera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY);         // no autofocusing
        parameters.setAutoExposureLock(false);                                  // keep the white balance constant
        camera.setParameters(parameters);
        camera.setDisplayOrientation(90);                                       // rotate to portrait mode

        try {
            camera.setPreviewTexture(surface);
            camera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        camera.stopPreview();
        camera.release();
        return true;
    }

    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        camTextureView.getBitmap(bmp);

        final Canvas c = camSurfaceHolder.lockCanvas();
        int lineLoc = 0;

        if (c != null) {
            int[] pixels = new int[bmp.getWidth()];                         // pixels[] is the RGBA data
            int row = 240;                                                  // Measure at the center row

            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);

            int sum_mr = 0;                                                 // the sum of the mass times the radius
            int sum_m = 0;                                                  // the sum of the masses

            for (int i = 0; i < bmp.getWidth(); i++) {
                if((((red(pixels[i]) - (green(pixels[i]) + blue(pixels[i])) / 2) > -redThreshold) && (red(pixels[i]) - (green(pixels[i]) + blue(pixels[i])) / 2) < redThreshold) && (red(pixels[i]) > redAlpha)) {
                    pixels[i] = rgb(1, 1, 1);                // set the pixel to near black
                }

                sum_m = sum_m + green(pixels[i]) + red(pixels[i]) + blue(pixels[i]);
                sum_mr = sum_mr + (green(pixels[i]) + red(pixels[i]) + blue(pixels[i])) * i;
            }

            // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
            if (sum_m > 5) {
                lineLoc = sum_mr / sum_m;
            }
            else {
                lineLoc = 0;                                                  // Assume COM is center (this will be averaged out)
            }

            // Update the row
            bmp.setPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
        }

        // Send the COM value over the USB port to the microcontroller
        String sendString = String.valueOf(lineLoc) + '\n';
        try {
            sPort.write(sendString.getBytes(), 10); // 10 is the timeout
        } catch (IOException e) { }

        // Draw a circle at the COM
        canvas.drawCircle(lineLoc, 240, 10, paint1);                        // x position, y position, diameter, color

        // Update the bitmap
        c.drawBitmap(bmp, 0, 0, null);
        camSurfaceHolder.unlockCanvasAndPost(c);

        // Calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        camTextView.setText("FPS " + 1000 / diff);
        prevtime = nowtime;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {

                }

                @Override
                public void onNewData(final byte[] data) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            MainActivity.this.updateReceivedData(data);
                        }
                    });
                }
            };

    @Override
    protected void onPause(){
        super.onPause();
        stopIoManager();
        if(sPort != null){
            try{
                sPort.close();
            } catch (IOException e){ }
            sPort = null;
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

        ProbeTable customTable = new ProbeTable();
        customTable.addProduct(0x04D8,0x000A, CdcAcmSerialDriver.class);
        UsbSerialProber prober = new UsbSerialProber(customTable);

        final List<UsbSerialDriver> availableDrivers = prober.findAllDrivers(manager);

        if(availableDrivers.isEmpty()) {
            //check
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        sPort = driver.getPorts().get(0);

        if (sPort == null){
            //check
        }else{
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null){
                //check
                PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent("com.android.example.USB_PERMISSION"), 0);
                usbManager.requestPermission(driver.getDevice(), pi);
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            } catch (IOException e) {
                //check
                try{
                    sPort.close();
                } catch (IOException e1) { }
                sPort = null;
                return;
            }
        }
        onDeviceStateChange();
    }

    private void stopIoManager(){
        if(mSerialIoManager != null) {
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if(sPort != null){
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange(){
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        //do something with received data

        //for displaying:
        String rxString = null;
        try {
            rxString = new String(data, "UTF-8");                    // put the data you got into a string
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }
}
