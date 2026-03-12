package com.xcup.bletest;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "BleTest";

    // ======================== BLE UUIDs (与iOS保持一致) ========================
    private static final UUID SERVICE_UUID =
            UUID.fromString("e43c4cbf-9e30-44cc-b8ea-83561908a4e5");
    private static final UUID RX_CHAR_UUID =   // App->Dev
            UUID.fromString("71cd6e15-8ed6-4727-b306-42a0e20fe7b6");
    private static final UUID TX_CHAR_UUID =   // Dev->App
            UUID.fromString("2cbb355f-d59a-4be6-aaab-fcfe27abcec4");
    private static final UUID CCCD_UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    // ======================== 设备识别 ========================
    private static final String TARGET_NAME_PREFIX = "XCUP-A1B2";

    // ======================== 协议常量 ========================
    private static final byte VER           = 0x01;
    private static final byte CMD_SET_PATTERN = 0x04;
    private static final byte CMD_STOP_ALL    = 0x02;
    private static final byte CMD_QUERY_STATE = 0x03;
    private static final byte CMD_STATE_RPT   = (byte) 0x83;
    private static final byte CMD_HEARTBEAT   = 0x06;
    private static final byte CMD_RESUME_APP  = 0x12;  // 恢复App控制

    // StateReport扩展解析用常量
    private static final byte SRC_FW     = 0;
    private static final byte SRC_APP    = 1;
    private static final byte SRC_BUTTON = 2;
    private static final byte SRC_SAFETY = 3;

    private static final byte OWNER_IDLE  = 0;
    private static final byte OWNER_APP   = 1;
    private static final byte OWNER_LOCAL = 2;

    private static final byte HOLD_NONE   = 0;
    private static final byte HOLD_TIMED  = 1;
    private static final byte HOLD_MANUAL = 2;

    // 模式和强度定义
    private static final byte PATTERN_1 = 1;
    private static final byte PATTERN_2 = 2;
    private static final byte PATTERN_3 = 3;

    private static final byte LEVEL_STOP = 0;
    private static final byte LEVEL_L    = 1;
    private static final byte LEVEL_M    = 2;
    private static final byte LEVEL_H    = 3;

    // ======================== 权限请求码 ========================
    private static final int REQUEST_PERMISSIONS = 1001;

    // ======================== UI Elements ========================
    private Button btnConnect;
    private Button btnResume;
    private TextView lblConnStatus;
    private TextView lblNeedAction;
    private TextView lblLastAction;

    // ======================== BLE Properties ========================
    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner bleScanner;
    private BluetoothGatt bluetoothGatt;
    private BluetoothGattCharacteristic rxCharacteristic;
    private BluetoothGattCharacteristic txCharacteristic;

    // ======================== State ========================
    private volatile boolean isConnected = false;
    private volatile boolean isScanning = false;
    private volatile boolean pausedByLocal = false;  // 暂停标志
    private String nextAction = "—";
    private String lastAction = "—";
    private byte seq = 0;

    // 顺序动作索引（0~3 对应 001~004）
    private int actionIndex = 0;

    // ======================== Handler & Runnables ========================
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    private final Runnable actionRunnable = new Runnable() {
        @Override
        public void run() {
            executeActionCycle();
            mainHandler.postDelayed(this, 5000); // 每5秒
        }
    };

    private final Runnable uiUpdateRunnable = new Runnable() {
        @Override
        public void run() {
            updateUI();
            mainHandler.postDelayed(this, 500); // 每0.5秒
        }
    };

    private final Runnable scanTimeoutRunnable = new Runnable() {
        @Override
        public void run() {
            stopScan();
            runOnUiThread(() -> lblConnStatus.setText("连接状态：扫描超时"));
        }
    };

    // ======================== Lifecycle ========================
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        setupUI();
        setupBLE();
        checkAndRequestPermissions();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mainHandler.removeCallbacks(actionRunnable);
        mainHandler.removeCallbacks(uiUpdateRunnable);
        mainHandler.removeCallbacks(scanTimeoutRunnable);
        disconnect();
    }

    // ======================== Setup ========================
    private void setupUI() {
        btnConnect   = findViewById(R.id.btnConnect);
        btnResume    = findViewById(R.id.btnResume);
        lblConnStatus = findViewById(R.id.lblConnStatus);
        lblNeedAction = findViewById(R.id.lblNeedAction);
        lblLastAction = findViewById(R.id.lblLastAction);

        btnConnect.setText("连接 / 断开");
        btnConnect.setOnClickListener(v -> connectButtonTapped());

        // 设置恢复按钮
        btnResume.setText("已暂停app操作，点击恢复");
        btnResume.setVisibility(View.GONE); // 默认隐藏
        btnResume.setOnClickListener(v -> resumeButtonTapped());

        updateUI();
    }

    private void setupBLE() {
        BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        if (bluetoothManager != null) {
            bluetoothAdapter = bluetoothManager.getAdapter();
        }

        if (bluetoothAdapter != null) {
            bleScanner = bluetoothAdapter.getBluetoothLeScanner();
        }
    }

    // ======================== 权限处理 ========================
    private void checkAndRequestPermissions() {
        List<String> permissionsNeeded = new ArrayList<>();

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // Android 12+
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN)
                    != PackageManager.PERMISSION_GRANTED) {
                permissionsNeeded.add(Manifest.permission.BLUETOOTH_SCAN);
            }
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                permissionsNeeded.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
        } else {
            // Android 11及以下
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    != PackageManager.PERMISSION_GRANTED) {
                permissionsNeeded.add(Manifest.permission.ACCESS_FINE_LOCATION);
            }
        }

        if (!permissionsNeeded.isEmpty()) {
            ActivityCompat.requestPermissions(this,
                    permissionsNeeded.toArray(new String[0]),
                    REQUEST_PERMISSIONS);
        } else {
            onPermissionsGranted();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_PERMISSIONS) {
            boolean allGranted = true;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    allGranted = false;
                    break;
                }
            }
            if (allGranted) {
                onPermissionsGranted();
            } else {
                Toast.makeText(this, "需要蓝牙权限才能使用", Toast.LENGTH_LONG).show();
            }
        }
    }

    private void onPermissionsGranted() {
        startActionLoop();
        startUIUpdateLoop();
    }

    // ======================== UI Actions ========================
    private void connectButtonTapped() {
        if (isConnected) {
            disconnect();
        } else {
            startScanAndConnect();
        }
    }

    // 恢复按钮点击事件
    private void resumeButtonTapped() {
        sendResumeAppControl();
    }

    // ======================== UI Updates ========================
    private void startUIUpdateLoop() {
        mainHandler.post(uiUpdateRunnable);
    }

    private void updateUI() {
        runOnUiThread(() -> {
            lblConnStatus.setText("连接状态：" + (isConnected ? "已连接" : "未连接"));
            lblNeedAction.setText("需要发送动作：" + nextAction);
            lblLastAction.setText("最近发送动作：" + lastAction);
            // 根据暂停状态显示/隐藏恢复按钮
            btnResume.setVisibility(pausedByLocal ? View.VISIBLE : View.GONE);
        });
    }

    // 设置暂停状态
    private void setPaused(boolean paused) {
        pausedByLocal = paused;
        runOnUiThread(() -> btnResume.setVisibility(paused ? View.VISIBLE : View.GONE));
    }

    // ======================== Action Generator ========================
    private void startActionLoop() {
        // 立即执行一次
        executeActionCycle();
        mainHandler.postDelayed(actionRunnable, 5000);
    }

    private void executeActionCycle() {
        String[] actions = {"001", "002", "003", "004"};
        // 顺序循环
        nextAction = actions[actionIndex];
        actionIndex = (actionIndex + 1) % actions.length;

        // 仅在已连接且未暂停时发送
        if (isConnected && rxCharacteristic != null && !pausedByLocal) {
            byte[] frame = buildFrameForAction(nextAction);
            if (frame != null) {
                writeToRx(frame);
                lastAction = nextAction;
            }
        }
    }

    // ======================== BLE Scanning ========================
    private void startScanAndConnect() {
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled()) {
            Log.w(TAG, "蓝牙未开启");
            return;
        }

        if (isScanning) {
            return;
        }

        isScanning = true;
        runOnUiThread(() -> lblConnStatus.setText("连接状态：扫描中…"));

        // 按Service UUID过滤扫描
        ScanFilter filter = new ScanFilter.Builder()
                .setServiceUuid(new ParcelUuid(SERVICE_UUID))
                .build();
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();

        try {
            if (bleScanner != null) {
                bleScanner.startScan(
                        Arrays.asList(filter), settings, scanCallback);
            }
        } catch (SecurityException e) {
            Log.e(TAG, "扫描权限不足", e);
            return;
        }

        // 20秒超时
        mainHandler.removeCallbacks(scanTimeoutRunnable);
        mainHandler.postDelayed(scanTimeoutRunnable, 20000);
    }

    private void stopScan() {
        if (isScanning) {
            try {
                if (bleScanner != null) {
                    bleScanner.stopScan(scanCallback);
                }
            } catch (SecurityException e) {
                Log.e(TAG, "停止扫描权限不足", e);
            }
            isScanning = false;
            mainHandler.removeCallbacks(scanTimeoutRunnable);
        }
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice device = result.getDevice();
            String name = result.getScanRecord() != null
                    ? result.getScanRecord().getDeviceName() : null;
            if (name == null) {
                try {
                    name = device.getName();
                } catch (SecurityException e) {
                    name = "";
                }
            }
            if (name == null) name = "";

            if (name.startsWith(TARGET_NAME_PREFIX)) {
                Log.i(TAG, "发现目标设备: " + name);
                stopScan();
                runOnUiThread(() -> lblConnStatus.setText("连接状态：连接中…"));
                try {
                    bluetoothGatt = device.connectGatt(
                            MainActivity.this, false, gattCallback,
                            BluetoothDevice.TRANSPORT_LE);
                } catch (SecurityException e) {
                    Log.e(TAG, "连接权限不足", e);
                }
            }
        }

        @Override
        public void onScanFailed(int errorCode) {
            Log.e(TAG, "扫描失败，errorCode=" + errorCode);
            isScanning = false;
        }
    };

    // ======================== BLE Connection ========================
    private void disconnect() {
        if (bluetoothGatt != null) {
            try {
                bluetoothGatt.disconnect();
                bluetoothGatt.close();
            } catch (SecurityException e) {
                Log.e(TAG, "断开权限不足", e);
            }
            bluetoothGatt = null;
        }
        isConnected = false;
        rxCharacteristic = null;
        txCharacteristic = null;
        setPaused(false); // 断开时清除暂停状态
        updateUI();
    }

    // ======================== GATT Callback ========================
    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                Log.i(TAG, "已连接到设备");
                isConnected = true;
                runOnUiThread(() -> lblConnStatus.setText("连接状态：已连接，协商MTU…"));

                // 请求MTU 247
                try {
                    gatt.requestMtu(247);
                } catch (SecurityException e) {
                    Log.e(TAG, "requestMtu权限不足", e);
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                Log.i(TAG, "设备断开连接");
                isConnected = false;
                rxCharacteristic = null;
                txCharacteristic = null;
                setPaused(false); // 断开时清除暂停状态
                try {
                    gatt.close();
                } catch (SecurityException e) {
                    Log.e(TAG, "close权限不足", e);
                }
                bluetoothGatt = null;
                updateUI();
            }
        }

        @Override
        public void onMtuChanged(BluetoothGatt gatt, int mtu, int status) {
            Log.i(TAG, "MTU协商结果: " + mtu + " status=" + status);
            runOnUiThread(() -> lblConnStatus.setText("连接状态：已连接，发现服务中…"));

            // 发现服务
            try {
                gatt.discoverServices();
            } catch (SecurityException e) {
                Log.e(TAG, "discoverServices权限不足", e);
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "发现服务失败, status=" + status);
                return;
            }

            BluetoothGattService service = gatt.getService(SERVICE_UUID);
            if (service == null) {
                Log.e(TAG, "未找到目标服务");
                return;
            }

            Log.i(TAG, "发现目标服务");

            // 获取RX特征
            rxCharacteristic = service.getCharacteristic(RX_CHAR_UUID);
            if (rxCharacteristic != null) {
                Log.i(TAG, "发现RX特征");
            }

            // 获取TX特征并订阅通知
            txCharacteristic = service.getCharacteristic(TX_CHAR_UUID);
            if (txCharacteristic != null) {
                Log.i(TAG, "发现TX特征");

                try {
                    // 开启本地通知
                    gatt.setCharacteristicNotification(txCharacteristic, true);

                    // 写CCCD描述符开启远端Notify
                    BluetoothGattDescriptor descriptor =
                            txCharacteristic.getDescriptor(CCCD_UUID);
                    if (descriptor != null) {
                        descriptor.setValue(
                                BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                        gatt.writeDescriptor(descriptor);
                    }
                } catch (SecurityException e) {
                    Log.e(TAG, "订阅通知权限不足", e);
                }
            }

            if (rxCharacteristic != null && txCharacteristic != null) {
                runOnUiThread(() ->
                        lblConnStatus.setText("连接状态：已连接（已订阅）"));
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt gatt,
                                      BluetoothGattDescriptor descriptor, int status) {
            if (descriptor.getUuid().equals(CCCD_UUID)) {
                if (status == BluetoothGatt.GATT_SUCCESS) {
                    Log.i(TAG, "已开启TX通知");
                } else {
                    Log.e(TAG, "写CCCD失败, status=" + status);
                }
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            if (characteristic.getUuid().equals(TX_CHAR_UUID)) {
                byte[] data = characteristic.getValue();
                Log.i(TAG, "收到数据: " + bytesToHex(data));
                parseIncomingFrame(data);
            }
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt gatt,
                                          BluetoothGattCharacteristic characteristic,
                                          int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.i(TAG, "写入成功");
            } else {
                Log.e(TAG, "写入失败, status=" + status);
            }
        }
    };

    // ======================== BLE Write ========================
    private void writeToRx(byte[] data) {
        if (bluetoothGatt == null || rxCharacteristic == null) return;

        rxCharacteristic.setValue(data);

        // 使用Write Without Response以减少延迟
        int props = rxCharacteristic.getProperties();
        if ((props & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0) {
            rxCharacteristic.setWriteType(
                    BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
        } else {
            rxCharacteristic.setWriteType(
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        }

        try {
            bluetoothGatt.writeCharacteristic(rxCharacteristic);
            Log.i(TAG, "发送数据: " + bytesToHex(data));
        } catch (SecurityException e) {
            Log.e(TAG, "写入权限不足", e);
        }
    }

    // ======================== Frame Building ========================
    private byte[] buildFrameForAction(String action) {
        switch (action) {
            case "001": // 恒定-低，循环
                return buildSetPatternFrame(PATTERN_1, (byte) 9, (short) 0, (byte) 1);
            case "002": // 脉冲-中，循环
                return buildSetPatternFrame(PATTERN_1, (byte) 5, (short) 0, (byte) 1);
            case "003": // 波形-中，循环
                return buildSetPatternFrame(PATTERN_1, (byte) 1, (short) 0, (byte) 1);
            case "004": // 模式1-强度3，持续2s
                return buildStopAllFrame();
            default:
                return null;
        }
    }

    private byte[] buildSetPatternFrame(byte patternId, byte intLevel,
                                        short durationMs, byte flags) {
        byte[] payload = new byte[5];
        payload[0] = patternId;
        payload[1] = intLevel;
        payload[2] = (byte) (durationMs & 0xFF);        // 小端低字节
        payload[3] = (byte) ((durationMs >> 8) & 0xFF);  // 小端高字节
        payload[4] = flags;

        return buildFrame(CMD_SET_PATTERN, payload);
    }

    private byte[] buildStopAllFrame() {
        return buildFrame(CMD_STOP_ALL, new byte[0]);
    }

    // 构建恢复控制帧
    private byte[] buildResumeFrame() {
        return buildFrame(CMD_RESUME_APP, new byte[0]);
    }

    private byte[] buildFrame(byte cmd, byte[] payload) {
        int payloadLen = payload.length;
        // 帧长度: SOF(2) + VER(1) + CMD(1) + SEQ(1) + LEN(2) + PAYLOAD + CRC(2)
        byte[] frame = new byte[2 + 1 + 1 + 1 + 2 + payloadLen + 2];
        int idx = 0;

        // SOF
        frame[idx++] = (byte) 0xAA;
        frame[idx++] = 0x55;

        // VER
        frame[idx++] = VER;

        // CMD
        frame[idx++] = cmd;

        // SEQ
        frame[idx++] = seq;

        // LEN (小端)
        frame[idx++] = (byte) (payloadLen & 0xFF);
        frame[idx++] = (byte) ((payloadLen >> 8) & 0xFF);

        // PAYLOAD
        System.arraycopy(payload, 0, frame, idx, payloadLen);
        idx += payloadLen;

        // CRC计算 (覆盖 VER..PAYLOAD)
        byte[] crcData = new byte[1 + 1 + 1 + 2 + payloadLen];
        int ci = 0;
        crcData[ci++] = VER;
        crcData[ci++] = cmd;
        crcData[ci++] = seq;
        crcData[ci++] = (byte) (payloadLen & 0xFF);
        crcData[ci++] = (byte) ((payloadLen >> 8) & 0xFF);
        System.arraycopy(payload, 0, crcData, ci, payloadLen);

        int crc = calculateCRC16CCITT(crcData);
        frame[idx++] = (byte) (crc & 0xFF);        // CRC小端低字节
        frame[idx++] = (byte) ((crc >> 8) & 0xFF);  // CRC小端高字节

        seq = (byte) ((seq + 1) & 0xFF); // 自增，溢出后自动回0

        return frame;
    }

    // 发送恢复App控制命令
    private void sendResumeAppControl() {
        if (!isConnected || rxCharacteristic == null) return;

        byte[] frame = buildResumeFrame();
        writeToRx(frame);
        // 等待ACK或StateReport确认恢复
    }

    // ======================== CRC16-CCITT ========================
    private int calculateCRC16CCITT(byte[] data) {
        int crc = 0xFFFF;

        for (byte b : data) {
            crc ^= (b & 0xFF) << 8;

            for (int i = 0; i < 8; i++) {
                if ((crc & 0x8000) != 0) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
                crc &= 0xFFFF;
            }
        }

        return crc;
    }

    // ======================== Frame Parsing ========================
    private void parseIncomingFrame(byte[] data) {
        if (data == null || data.length < 9) return; // 最小帧长度

        int index = 0;

        // SOF
        if ((data[index] & 0xFF) != 0xAA || (data[index + 1] & 0xFF) != 0x55) return;
        index += 2;

        // VER
        byte ver = data[index++];

        // CMD
        byte cmd = data[index++];

        // SEQ
        byte rseq = data[index++];

        // LEN (小端)
        int len = (data[index] & 0xFF) | ((data[index + 1] & 0xFF) << 8);
        index += 2;

        // 检查数据长度
        if (data.length < index + len + 2) return;

        // PAYLOAD
        byte[] payload = new byte[len];
        System.arraycopy(data, index, payload, 0, len);
        index += len;

        // CRC (小端)
        int receivedCrc = (data[index] & 0xFF) | ((data[index + 1] & 0xFF) << 8);

        // 验证CRC
        byte[] crcData = new byte[1 + 1 + 1 + 2 + len];
        int ci = 0;
        crcData[ci++] = ver;
        crcData[ci++] = cmd;
        crcData[ci++] = rseq;
        crcData[ci++] = (byte) (len & 0xFF);
        crcData[ci++] = (byte) ((len >> 8) & 0xFF);
        System.arraycopy(payload, 0, crcData, ci, len);

        int calculatedCrc = calculateCRC16CCITT(crcData);

        if (receivedCrc != calculatedCrc) {
            Log.e(TAG, "CRC校验失败");
            return;
        }

        // 处理命令
        int cmdUnsigned = cmd & 0xFF;

        if (cmdUnsigned == (0x80 + (CMD_SET_PATTERN & 0xFF)) ||
                cmdUnsigned == (0x80 + (CMD_STOP_ALL & 0xFF))) {
            // ACK
            if (payload.length >= 2) {
                byte ackSeq = payload[0];
                byte status = payload[1];
                Log.i(TAG, String.format("收到ACK: cmd=0x%02X seq=%d status=%d",
                        cmdUnsigned, ackSeq & 0xFF, status & 0xFF));

                // 锁定期：ACK=BUSY -> 进入暂停并提示
                if (status == 1) { // BUSY
                    setPaused(true);
                }

                // 恢复命令成功 -> 清暂停
                if (cmdUnsigned == (0x80 + (CMD_RESUME_APP & 0xFF)) && status == 0) {
                    setPaused(false);
                }
            }
        } else if (cmdUnsigned == (CMD_STATE_RPT & 0xFF)) {
            Log.i(TAG, "收到状态报告，长度=" + payload.length);
            parseStateReport(payload);
        } else {
            Log.i(TAG, String.format("收到命令: 0x%02X 长度=%d", cmdUnsigned, len));
        }
    }

    // 解析StateReport扩展字段
    private void parseStateReport(byte[] payload) {
        if (payload.length < 12) return;

        int offset = 0;

        // 跳过基础字段
        offset += 2; // FW_VER
        offset += 2; // BAT_mV
        offset += 2; // TEMP_dC
        offset += 1; // CH_CNT
        offset += 1; // CUR_PATTERN
        offset += 1; // CUR_INTLVL
        offset += 2; // RUN_REMAIN
        offset += 1; // FLAGS

        // 解析扩展字段（至少9字节）
        if (payload.length < offset + 9) return;

        int rev = (payload[offset] & 0xFF) | ((payload[offset + 1] & 0xFF) << 8);
        offset += 2;

        byte src = payload[offset++];
        byte chgMask = payload[offset++];
        byte owner = payload[offset++];
        byte holdMode = payload[offset++];

        int holdTtl = (payload[offset] & 0xFF) | ((payload[offset + 1] & 0xFF) << 8);
        offset += 2;

        byte btnCode = payload[offset++];

        Log.i(TAG, String.format("StateReport: src=%d owner=%d holdMode=%d holdTtl=%d",
                src, owner, holdMode, holdTtl));

        // 根据状态更新暂停标志
        if (src == SRC_BUTTON && holdMode == HOLD_MANUAL) {
            // 本地按键触发手动锁定
            setPaused(true);
        }

        if (holdMode == HOLD_NONE) {
            // 无锁定，恢复正常
            setPaused(false);
        }
    }

    // ======================== Helper ========================
    private static String bytesToHex(byte[] bytes) {
        if (bytes == null) return "null";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < bytes.length; i++) {
            if (i > 0) sb.append(' ');
            sb.append(String.format("%02X", bytes[i] & 0xFF));
        }
        return sb.toString();
    }
}
