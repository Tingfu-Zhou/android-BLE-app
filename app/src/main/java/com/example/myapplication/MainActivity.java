package com.example.myapplication;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.*;
import android.bluetooth.le.*;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.*;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresPermission;
import androidx.appcompat.app.AppCompatActivity;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.util.concurrent.*;

public class MainActivity extends AppCompatActivity {

    // ====== TODO: 依《BLE 通信协议 v1.0》填写 UUID（必须与固件一致）======
    private static final UUID SERVICE_UUID = UUID.fromString("e43c4cbf-9e30-44cc-b8ea-83561908a4e5"); // TODO: 替换为最终值
    private static final UUID RX_CHAR_UUID = UUID.fromString("71cd6e15-8ed6-4727-b306-42a0e20fe7b6"); // App->Dev (Write/WWR)
    private static final UUID TX_CHAR_UUID = UUID.fromString("2cbb355f-d59a-4be6-aaab-fcfe27abcec4"); // Dev->App (Notify)
    private static final UUID CCCD_UUID    = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    // ====== TODO: 指定目标设备标识（建议使用广播名前缀或 Service UUID 过滤）======
    private static final String TARGET_NAME_PREFIX = "XCUP-A1B2";      // 示例：XCUP-A1B2
    // 如已知 MAC，可直接使用（Android 12+ 无法直接扫描到随机地址，建议用名称/Service UUID）
    // private static final String TARGET_MAC = "00:11:22:33:44:55";

    private static final String TAG = "BLETest";
    private static final int    VER = 0x01; // 帧版本
    // CMD 定义（与协议一致）
    private static final int CMD_SET_PATTERN = 0x04;
    private static final int CMD_STOP_ALL    = 0x02;
    private static final int CMD_QUERY_STATE = 0x03;
    private static final int CMD_STATE_RPT   = 0x83; // Dev->App
    private static final int CMD_HEARTBEAT   = 0x06;
    private static final int CMD_RESUME_APP  = 0x12;  // [ADD] 恢复App控制

    // [ADD] StateReport 扩展解析用常量（与协议一致）
    private static final int SRC_FW = 0, SRC_APP = 1, SRC_BUTTON = 2, SRC_SAFETY = 3;
    private static final int OWNER_IDLE = 0, OWNER_APP = 1, OWNER_LOCAL = 2;
    private static final int HOLD_NONE = 0, HOLD_TIMED = 1, HOLD_MANUAL = 2;

    // ====== UI ======
    private TextView tvConn, tvNeed, tvLast;
    private Button   btnConnect;
    private Button   btnResume; // [ADD] 顶部“恢复”按钮

    // ====== BLE ======
    private BluetoothAdapter      adapter;
    private BluetoothLeScanner    scanner;
    private BluetoothGatt         gatt;
    private BluetoothGattCharacteristic rxChar, txChar;
    private boolean               isConnected = false;
    private boolean               isNotifying = false;

    // ====== 调度：随机动作每 10 秒生成一次 ======
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private ScheduledFuture<?> randomTask;
    private volatile String nextAction = "—";  // “需要发送动作”
    private volatile String lastAction = "—";  // “最近发送动作”
    private final Random rand = new Random();

    // [ADD] 暂停标志：本地按键触发“手动恢复”模式时置 true
    private volatile boolean pausedByLocal = false;

    // ====== Android 12+ 权限 ======
    private ActivityResultLauncher<String[]> permLauncher;
    private boolean pendingConnectRequest = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        tvConn  = findViewById(R.id.tvConn);
        tvNeed  = findViewById(R.id.tvNeed);
        tvLast  = findViewById(R.id.tvLast);
        btnConnect = findViewById(R.id.btnConnect);
        btnResume  = findViewById(R.id.btnResume); // [ADD]

        BluetoothManager mgr = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        adapter = mgr.getAdapter();
        scanner = adapter != null ? adapter.getBluetoothLeScanner() : null;

        permLauncher = registerForActivityResult(new ActivityResultContracts.RequestMultiplePermissions(), result -> {
            boolean granted = areBlePermissionsGranted();
            if (pendingConnectRequest) {
                pendingConnectRequest = false;
                if (granted) {
                    startScanAndConnect();
                }
            }
        });

        btnConnect.setOnClickListener(v -> {
            if (!isConnected) {
                if (requestBlePermissions()) {
                    startScanAndConnect();
                } else {
                    pendingConnectRequest = true;
                }
            } else {
                disconnectGatt();
            }
        });

        // [ADD] 恢复按钮：发 ResumeAppControl
        btnResume.setOnClickListener(v -> {
            sendResumeAppControl();
        });

        // 启动 10 秒随机动作任务
        startRandomActionLoop();
        // 启动 UI 刷新（每 500ms）
        new Handler(Looper.getMainLooper()).postDelayed(new Runnable() {
            @Override public void run() {
                tvNeed.setText("需要发送动作：" + nextAction);
                tvLast.setText("最近发送动作：" + lastAction);
                tvConn.setText("连接状态：" + (isConnected ? "已连接" : "未连接"));
                btnResume.setVisibility(pausedByLocal ? View.VISIBLE : View.GONE); // [ADD]
                new Handler(Looper.getMainLooper()).postDelayed(this, 500);
            }
        }, 500);
    }

    private boolean requestBlePermissions() {
        List<String> req = new ArrayList<>();
        if (!areBlePermissionsGranted(req)) {
            permLauncher.launch(req.toArray(new String[0]));
            return false;
        }
        return true;
    }

    private boolean areBlePermissionsGranted() {
        return areBlePermissionsGranted(null);
    }

    private boolean areBlePermissionsGranted(List<String> collector) {
        List<String> target = collector != null ? collector : new ArrayList<>();
        if (Build.VERSION.SDK_INT >= 31) {
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED)
                target.add(Manifest.permission.BLUETOOTH_SCAN);
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED)
                target.add(Manifest.permission.BLUETOOTH_CONNECT);
        } else {
            if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED)
                target.add(Manifest.permission.ACCESS_FINE_LOCATION);
        }
        return target.isEmpty();
    }

    // === 扫描并自动连接到指定设备（名称前缀或 Service 过滤）===
    @SuppressLint("MissingPermission")
    private void startScanAndConnect() {
        if (scanner == null || adapter == null || !adapter.isEnabled()) return;

        tvConn.setText("连接状态：扫描中…");

        List<ScanFilter> filters = new ArrayList<>();
        // 通过 Service UUID 过滤更稳妥（建议固件广播 Service UUID）
        filters.add(new ScanFilter.Builder().setServiceUuid(new ParcelUuid(SERVICE_UUID)).build());

        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();

        scanner.startScan(filters, settings, scanCallback);

        // 防止一直扫：20 秒超时
        new Handler(Looper.getMainLooper()).postDelayed(() -> {
            try { scanner.stopScan(scanCallback); } catch (Exception ignore) {}
        }, 20_000);
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        @Override public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice dev = result.getDevice();
            String name = result.getScanRecord() != null ? result.getScanRecord().getDeviceName() : dev.getName();
            if (name != null && name.startsWith(TARGET_NAME_PREFIX)) {
                Log.i(TAG, "Found target: " + name + " -> connecting…");
                stopScanSafe();
                connectGatt(dev);
            }
        }
        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        @Override public void onBatchScanResults(List<ScanResult> results) {
            for (ScanResult r : results) onScanResult(0, r);
        }
        @Override public void onScanFailed(int errorCode) {
            Log.e(TAG, "Scan failed: " + errorCode);
        }
    };

    @SuppressLint("MissingPermission")
    private void stopScanSafe() {
        try { if (scanner != null) scanner.stopScan(scanCallback); } catch (Exception ignore) {}
    }

    // === 连接 GATT 并执行：requestMtu(247) -> 发现服务 -> 订阅 Notify ===
    @SuppressLint("MissingPermission")
    private void connectGatt(BluetoothDevice dev) {
        gatt = dev.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
    }

    @SuppressLint("MissingPermission")
    private void disconnectGatt() {
        if (gatt != null) {
            gatt.disconnect();
            gatt.close();
            gatt = null;
        }
        isConnected = false;
        isNotifying = false;
        setPaused(false); // [ADD] 断开时清暂停
        tvConn.setText("连接状态：未连接");
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        @Override public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            Log.i(TAG, "onConnectionStateChange: status=" + status + ", newState=" + newState);
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                isConnected = true;
                runOnUiThread(() -> tvConn.setText("连接状态：已连接，协商 MTU…"));
                if (Build.VERSION.SDK_INT >= 21) {
                    g.requestMtu(247);
                } else {
                    g.discoverServices();
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                isConnected = false;
                isNotifying = false;
                runOnUiThread(() -> tvConn.setText("连接状态：已断开"));
                setPaused(false); // [ADD]
            }
        }

        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        @Override public void onMtuChanged(BluetoothGatt g, int mtu, int status) {
            Log.i(TAG, "onMtuChanged: mtu=" + mtu + ", status=" + status);
            g.discoverServices();
        }

        @Override public void onServicesDiscovered(BluetoothGatt g, int status) {
            BluetoothGattService svc = g.getService(SERVICE_UUID);
            if (svc == null) {
                Log.e(TAG, "Service not found");
                return;
            }
            rxChar = svc.getCharacteristic(RX_CHAR_UUID);
            txChar = svc.getCharacteristic(TX_CHAR_UUID);
            if (rxChar == null || txChar == null) {
                Log.e(TAG, "RX/TX characteristic missing");
                return;
            }
            enableNotify(g, txChar);
        }

        @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) {
            if (TX_CHAR_UUID.equals(c.getUuid())) {
                byte[] data = c.getValue();
                parseIncomingFrame(data);
            }
        }

        @Override public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int status) {
            Log.i(TAG, "Write status=" + status);
        }
    };

    @SuppressLint("MissingPermission")
    private void enableNotify(BluetoothGatt g, BluetoothGattCharacteristic ch) {
        boolean ok = g.setCharacteristicNotification(ch, true);
        BluetoothGattDescriptor cccd = ch.getDescriptor(CCCD_UUID);
        if (cccd != null) {
            cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            g.writeDescriptor(cccd);
        }
        isNotifying = ok;
        runOnUiThread(() -> tvConn.setText("连接状态：已连接（已订阅）"));
    }

    // === 启动“每 10 秒随机动作” ===
    private void startRandomActionLoop() {
        if (randomTask != null && !randomTask.isCancelled()) return;
        randomTask = scheduler.scheduleWithFixedDelay(() -> {
            String[] arr = {"001", "002", "003", "004"};
            String a = arr[rand.nextInt(arr.length)];
            nextAction = a; // 刷新 UI 由主线程轮询
            // [MOD] 仅在已连接且未暂停时才发送
            if (isConnected && rxChar != null && gatt != null && !pausedByLocal) {
                // 映射到协议命令并发送
                byte[] frame = buildFrameForAction(a);
                if (frame != null) {
                    writeRx(frame);
                    lastAction = a;
                }
            }
        }, 0, 10, TimeUnit.SECONDS);
    }

    @SuppressLint("MissingPermission")
    private void writeRx(byte[] frame) {
        if (rxChar == null || gatt == null) return;
        // 优先使用 Write Without Response（减少握手时延）；若固件未开，可切回 WRITE_TYPE_DEFAULT
        rxChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
        rxChar.setValue(frame);
        gatt.writeCharacteristic(rxChar);
    }

    private static final int PATTERN_1 = 1; // 模式1：恒定（示例）
    private static final int PATTERN_2 = 2; // 模式2：脉冲
    private static final int PATTERN_3  = 3; // 模式3：波形

    private static final int LEVEL_STOP = 0;
    private static final int LEVEL_L    = 1;    // 低
    private static final int LEVEL_M    = 2;    // 中
    private static final int LEVEL_H    = 3;    // 高

    private byte[] buildFrameForAction(String action) {
        switch (action) {
            // TODO：PATTERN_ID/INT_LEVEL/DURATION/FLAGS 仅为占位示例
            case "001": // 例如：恒定-低，持续2s
                return buildSetPatternFrame(PATTERN_1, LEVEL_L, 2000, 0);
            case "002": // 例如：脉冲-中，持续2s
                return buildSetPatternFrame(PATTERN_2, LEVEL_M, 2000, 0);
            case "003": // 例如：波形-中，循环（DURATION=0 表示持续）
                return buildSetPatternFrame(PATTERN_3, LEVEL_M, 0, 1 /*Loop*/);
            case "004": // 停止
                return buildStopAllFrame();
            default:
                return null;
        }
    }


    // === 帧编码(帧封装)：AA55 | VER | CMD | SEQ | LEN(le) | PAYLOAD | CRC16(le) ===
    private byte seq = 0;

    private byte[] buildSetPatternFrame(int patternId, int intLevel, int durationMs, int flags/*bit0 loop*/) {
        // PAYLOAD：PATTERN_ID(1) INT_LEVEL(1) DURATION_MS(2 le) FLAGS(1)
        ByteBuffer payload = ByteBuffer.allocate(1 + 1 + 2 + 1).order(ByteOrder.LITTLE_ENDIAN);
        payload.put((byte) patternId);
        payload.put((byte) intLevel);
        payload.putShort((short) durationMs);
        payload.put((byte) (flags & 0xFF));
        return buildFrame(CMD_SET_PATTERN, payload.array());
    }

    private byte[] buildStopAllFrame() {
        byte[] payload = new byte[0];
        return buildFrame(CMD_STOP_ALL, payload);
    }
    private byte[] buildResumeFrame() { // [ADD]
        return buildFrame(CMD_RESUME_APP, new byte[0]);
    }

    private byte[] buildFrame(int cmd, byte[] payload) {
        // LEN = payload length (le)
        int len = payload.length;
        ByteBuffer bb = ByteBuffer.allocate(2 + 1 + 1 + 1 + 2 + len + 2).order(ByteOrder.LITTLE_ENDIAN);
        bb.put((byte) 0xAA).put((byte) 0x55);        // SOF
        bb.put((byte) VER);                          // VER
        bb.put((byte) (cmd & 0xFF));                 // CMD
        bb.put((byte) (seq & 0xFF));                 // SEQ
        bb.putShort((short) len);                    // LEN (le)
        bb.put(payload);                             // PAYLOAD
        // 计算 CRC16-CCITT(0x1021, init 0xFFFF) 覆盖 VER..PAYLOAD
        byte[] crcInput = new byte[1 + 1 + 1 + 2 + len];
        ByteBuffer tmp = ByteBuffer.wrap(crcInput).order(ByteOrder.LITTLE_ENDIAN);
        tmp.put((byte) VER).put((byte) (cmd & 0xFF)).put((byte) (seq & 0xFF)).putShort((short) len).put(payload);
        int crc = crc16Ccitt(tmp.array());
        bb.putShort((short) crc);                    // CRC (le)
        seq++;                                       // 自增
        return bb.array();
    }

    // === CRC16-CCITT (poly 0x1021, init 0xFFFF, xorout 0x0000, refin=false, refout=false) ===
    private static int crc16Ccitt(byte[] data) {
        int crc = 0xFFFF;
        for (byte b : data) {
            crc ^= ((b & 0xFF) << 8);
            for (int i = 0; i < 8; i++) {
                if ((crc & 0x8000) != 0) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
                crc &= 0xFFFF;
            }
        }
        return crc & 0xFFFF;
    }

    // === 解析设备上行帧（ACK/StateReport/Heartbeat）===
    private void parseIncomingFrame(byte[] pkt) {
        try {
            if (pkt == null || pkt.length < 2 + 1 + 1 + 1 + 2 + 2) return;
            ByteBuffer bb = ByteBuffer.wrap(pkt).order(ByteOrder.LITTLE_ENDIAN);
            byte sof0 = bb.get(), sof1 = bb.get();
            if (sof0 != (byte) 0xAA || sof1 != (byte) 0x55) return;
            int ver = bb.get() & 0xFF;
            int cmd = bb.get() & 0xFF;
            int rseq = bb.get() & 0xFF;
            int len  = bb.getShort() & 0xFFFF;
            if (len < 0 || len > 512 || pkt.length < 2 + 1 + 1 + 1 + 2 + len + 2) return;
            byte[] payload = new byte[len];
            bb.get(payload);
            int crcLe = bb.getShort() & 0xFFFF;

            // 校验 CRC
            ByteBuffer tmp = ByteBuffer.allocate(1 + 1 + 1 + 2 + len).order(ByteOrder.LITTLE_ENDIAN);
            tmp.put((byte) ver).put((byte) cmd).put((byte) rseq).putShort((short) len).put(payload);
            int crc = crc16Ccitt(tmp.array());
            if (crc != crcLe) {
                Log.w(TAG, "CRC mismatch");
                return;
            }

            if (cmd == (0x80 + CMD_SET_PATTERN)
                    || cmd == (0x80 + CMD_STOP_ALL)
                    || cmd == (0x80 + CMD_QUERY_STATE)
                    || cmd == (0x80 + CMD_RESUME_APP)) {
                // ACK：payload = SEQ(1) STATUS(1)
                if (payload.length >= 2) {
                    int ackSeq = payload[0] & 0xFF;
                    int status = payload[1] & 0xFF;
                    Log.i(TAG, String.format(Locale.US, "ACK cmd=0x%02X seq=%d status=%d", cmd, ackSeq, status));
                    // [ADD] 锁定期：ACK=BUSY -> 进入暂停态并提示
                    boolean isResumeAck = (cmd == (0x80 + CMD_RESUME_APP));
                    if (isResumeAck && status == 0) {
                        // [ADD] 恢复命令成功或设备上报已解除 → 清暂停
                        setPaused(false);
                    } else if (status == 1 /*BUSY*/) {
                        setPaused(true);
                    }
                }
            } else if (cmd == CMD_STATE_RPT) {
                parseStateReport(payload); // [ADD]
                // 可按协议解析 StateReport 字段
                Log.i(TAG, "StateReport len=" + payload.length);
            } else {
                Log.i(TAG, String.format(Locale.US, "RX cmd=0x%02X len=%d", cmd, len));
            }
        } catch (Exception e) {
            Log.e(TAG, "parseIncomingFrame error", e);
        }
    }

    // [ADD] 解析 StateReport 扩展字段，控制暂停/恢复 UI
    private void parseStateReport(byte[] p) {
        try {
            int off = 0;
            if (p.length < 12) return; // 先越界保护
            off += 2; // FW_VER
            off += 2; // BAT_mV
            off += 2; // TEMP_dC
            off += 1; // CH_CNT
            off += 1; // CUR_PATTERN
            off += 1; // CUR_INTLVL
            off += 2; // RUN_REMAIN
            off += 1; // FLAGS

            // 扩展最少 9 字节：REV(2), SRC(1), CHG_MASK(1), OWNER(1), HOLD_MODE(1), HOLD_TTL(2), BTN_CODE(1)
            if (p.length >= off + 9) {
                int rev       = ((p[off] & 0xFF) | ((p[off+1] & 0xFF) << 8)); off += 2;
                int src       =  (p[off++] & 0xFF);
                int chgMask   =  (p[off++] & 0xFF);
                int owner     =  (p[off++] & 0xFF);
                int holdMode  =  (p[off++] & 0xFF);
                int holdTtl   = ((p[off] & 0xFF) | ((p[off+1] & 0xFF) << 8)); off += 2;
                int btnCode   =  (p[off++] & 0xFF);

                Log.i(TAG, "StateReport: src=" + src + " owner=" + owner + " holdMode=" + holdMode + " holdTtl=" + holdTtl);
                if (src == SRC_BUTTON && holdMode == HOLD_MANUAL) {
                    setPaused(true);
                }
                if (holdMode == HOLD_NONE) {
                    setPaused(false);
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "parseStateReport error", e);
        }
    }

    // [ADD] 统一设置暂停与按钮可见性
    private void setPaused(boolean paused) {
        pausedByLocal = paused;
        runOnUiThread(() -> btnResume.setVisibility(paused ? View.VISIBLE : View.GONE));
    }

    // [ADD] 发送 ResumeAppControl
    private void sendResumeAppControl() {
        if (!isConnected || rxChar == null || gatt == null) return;
        byte[] frame = buildResumeFrame();
        writeRx(frame);
        // 等 ACK 或 SR 恢复；这里直接先把按钮保持显示，成功后 parse 中会隐藏
    }

    @Override protected void onDestroy() {
        super.onDestroy();
        if (randomTask != null) randomTask.cancel(true);
        scheduler.shutdownNow();
        disconnectGatt();
        stopScanSafe();
    }
}
