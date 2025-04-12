//Author: Austin Allen
//Motorized Furniture Dolly
//Flutter App

import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'BLE Dolly Controller',
      home: BLEControlPage(),
      debugShowCheckedModeBanner: false,
    );
  }
}

class BLEControlPage extends StatefulWidget {
  @override
  _BLEControlPageState createState() => _BLEControlPageState();
}

class _BLEControlPageState extends State<BLEControlPage> {
  bool? isConnected;
  bool showDeviceList = true;
  final String targetDeviceName = "ESP32_S3_BLE";
  BluetoothDevice? connectedDevice;
  BluetoothCharacteristic? bleCharacteristic;

  final ValueNotifier<double> displayedSpeed = ValueNotifier<double>(0.0);

  double verticalValue = 0.0;
  double horizontalValue = 0.0;

  final double factor1 = 0.8;
  final double factor2 = -0.2;
  Timer? joystickTimer;

  bool allowJoystickSend = true;
  bool isWriting = false;
  int lastSentY = 9999;
  int lastSentX = 9999;

  @override
  void initState() {
    super.initState();
    requestPermissions();

    joystickTimer = Timer.periodic(Duration(milliseconds: 50), (_) {
      if (allowJoystickSend && bleCharacteristic != null && isConnected == true) {
        final y = (verticalValue * factor1).round();
        final x = (horizontalValue * factor2).round();

        if (y != lastSentY || x != lastSentX) {
          sendJoystickData(y, x);
          lastSentY = y;
          lastSentX = x;
        }
      }
    });
  }

  @override
  void dispose() {
    joystickTimer?.cancel();
    displayedSpeed.dispose();
    super.dispose();
  }

  Future<void> requestPermissions() async {
    await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.locationWhenInUse,
    ].request();
  }

  void resetVertical() {
    setState(() {
      verticalValue = 0;
    });

    final y = (verticalValue * factor1).round();
    final x = (horizontalValue * factor2).round();
    sendJoystickData(y, x);
  }

  void resetHorizontal() {
    setState(() {
      horizontalValue = 0;
    });

    final y = (verticalValue * factor1).round();
    final x = (horizontalValue * factor2).round();
    sendJoystickData(y, x);
  }

  Future<void> _setupDevice(BluetoothDevice device) async {
    setState(() {
      isConnected = true;
    });

    List<BluetoothService> services = await device.discoverServices();
    for (BluetoothService service in services) {
      if (service.uuid.toString().toLowerCase() == "4fafc201-1fb5-459e-8fcc-c5c9c331914b") {
        for (BluetoothCharacteristic c in service.characteristics) {
          if (c.uuid.toString().toLowerCase() == "beb5483e-36e1-4688-b7f5-ea07361b26a8") {
            bleCharacteristic = c;
            if (c.properties.notify) {
              await c.setNotifyValue(true);
              c.lastValueStream.listen((value) {
                //print("BLE RAW: $value (len: ${value.length})");
                try {
                  final decoded = String.fromCharCodes(value).trim();
                  //print("BLE DECODED STRING: '$decoded'");

                  final rpm = double.tryParse(decoded) ?? 0.0;
                  final speed = rpm * 0.00192;

                  displayedSpeed.value = speed;
                } catch (e) {
                  print("BLE PARSE ERROR: $e");
                }
              });
            }
          }
        }
      }
    }
  }

  void scanAndConnect() async {
    await FlutterBluePlus.startScan(timeout: Duration(seconds: 4));
  }

  void sendJoystickData(int y, int x) async {
    if (bleCharacteristic != null && !isWriting) {
      final scaledY = y * 24;
      final scaledX = x * 24;
      final dataString = "$scaledY,$scaledX";
      try {
        isWriting = true;
        await bleCharacteristic!.write(dataString.codeUnits);
      } catch (_) {
        // skip if busy
      } finally {
        isWriting = false;
      }
    }
  }

  void sendCommand(String cmd) async {
    if (bleCharacteristic != null) {
      allowJoystickSend = false;
      await bleCharacteristic!.write(cmd.codeUnits);
      await Future.delayed(Duration(milliseconds: 200));
      allowJoystickSend = true;
    }
  }

  @override
  Widget build(BuildContext context) {
    String connectionStatus = "Disconnected";
    if (connectedDevice != null) {
      connectionStatus = isConnected == true
          ? "Connected to $targetDeviceName"
          : "Connecting...";
    }

    return Scaffold(
      body: SafeArea(
        child: Column(
          children: [
            Container(
              padding: EdgeInsets.all(8),
              alignment: Alignment.centerLeft,
              child: Row(
                children: [
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        showDeviceList = !showDeviceList;
                      });
                    },
                    child: Text(showDeviceList ? "Hide Devices" : "Show Devices"),
                  ),
                  SizedBox(width: 8),
                  ElevatedButton(
                    onPressed: scanAndConnect,
                    child: Text("Scan for Devices"),
                  ),
                  SizedBox(width: 16),
                  Icon(
                    isConnected == true
                        ? Icons.bluetooth_connected
                        : Icons.bluetooth_disabled,
                    color: isConnected == true ? Colors.green : Colors.red,
                  ),
                  SizedBox(width: 8),
                  Text(
                    connectionStatus,
                    style: TextStyle(fontSize: 16),
                  ),
                ],
              ),
            ),
            Expanded(
              child: Column(
                children: [
                  if (showDeviceList)
                    Expanded(
                      flex: 1,
                      child: StreamBuilder<List<ScanResult>>(
                        stream: FlutterBluePlus.scanResults,
                        builder: (context, snapshot) {
                          if (!snapshot.hasData || snapshot.data!.isEmpty) {
                            return Center(child: Text("No devices found"));
                          }
                          final results = snapshot.data!;
                          return ListView.builder(
                            scrollDirection: Axis.horizontal,
                            itemCount: results.length,
                            itemBuilder: (context, index) {
                              final device = results[index].device;
                              return Padding(
                                padding: const EdgeInsets.symmetric(horizontal: 8),
                                child: ElevatedButton(
                                  onPressed: () async {
                                    setState(() {
                                      isConnected = false;
                                      showDeviceList = false;
                                      connectedDevice = device;
                                    });

                                    await FlutterBluePlus.stopScan();

                                    try {
                                      await connectedDevice!.connect();
                                    } catch (e) {
                                      await Future.delayed(Duration(seconds: 1));
                                      try {
                                        await connectedDevice!.connect();
                                      } catch (e) {
                                        setState(() {
                                          isConnected = null;
                                          connectedDevice = null;
                                        });
                                        return;
                                      }
                                    }

                                    await _setupDevice(connectedDevice!);
                                  },
                                  child: Column(
                                    mainAxisSize: MainAxisSize.min,
                                    children: [
                                      Text(device.name.isNotEmpty
                                          ? device.name
                                          : device.id.toString()),
                                      Text("RSSI: ${results[index].rssi}",
                                          style: TextStyle(fontSize: 12)),
                                    ],
                                  ),
                                ),
                              );
                            },
                          );
                        },
                      ),
                    ),
                  Expanded(
                    flex: 3,
                    child: Row(
                      children: [
                        // Vertical Joystick (Left)
                        Expanded(
                          flex: 1,
                          child: Listener(
                            onPointerMove: (event) {
                              final localDelta = event.delta;
                              setState(() {
                                verticalValue -= localDelta.dy;
                                verticalValue = verticalValue.clamp(-2400.0, 2400.0);
                              });

                              final y = (verticalValue * factor1).round();
                              final x = (horizontalValue * factor2).round();
                              sendJoystickData(y, x);
                            },
                            onPointerUp: (_) => resetVertical(),
                            child: Container(
                              color: Colors.blue.shade50,
                              child: Center(
                                child: RotatedBox(
                                  quarterTurns: 3,
                                  child: Slider(
                                    value: verticalValue,
                                    min: -100,
                                    max: 100,
                                    onChanged: (_) {},
                                  ),
                                ),
                              ),
                            ),
                          ),
                        ),
                        // Center Info
                        Expanded(
                          flex: 2,
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              ValueListenableBuilder<double>(
                                valueListenable: displayedSpeed,
                                builder: (context, value, _) {
                                  return Text(
                                    "Speed: ${value.toStringAsFixed(2)} mph",
                                    style: TextStyle(fontSize: 24),
                                  );
                                },
                              ),
                              SizedBox(height: 20),
                              ElevatedButton(
                                onPressed: () => sendCommand("L"),
                                child: Icon(Icons.lightbulb),
                              ),
                            ],
                          ),
                        ),
                        // Horizontal Joystick (Right)
                        Expanded(
                          flex: 1,
                          child: Listener(
                            onPointerMove: (event) {
                              final localDelta = event.delta;
                              setState(() {
                                horizontalValue += localDelta.dx;
                                horizontalValue = horizontalValue.clamp(-2400.0, 2400.0);
                              });

                              final y = (verticalValue * factor1).round();
                              final x = (horizontalValue * factor2).round();
                              sendJoystickData(y, x);
                            },
                            onPointerUp: (_) => resetHorizontal(),
                            child: Container(
                              color: Colors.green.shade50,
                              child: Center(
                                child: Slider(
                                  value: horizontalValue,
                                  min: -100,
                                  max: 100,
                                  onChanged: (_) {},
                                ),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
