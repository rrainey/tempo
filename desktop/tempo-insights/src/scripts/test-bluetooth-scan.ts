// src/scripts/test-bluetooth-scan.ts
// Run with: tsx src/scripts/test-bluetooth-scan.ts

import { BluetoothService } from '../lib/bluetooth/bluetooth.service';

async function testBluetoothScanning() {
  console.log('=== Bluetooth Scanning Test ===\n');
  
  const bluetooth = BluetoothService.getInstance();
  
  // Check prerequisites
  console.log('1. Checking prerequisites...');
  
  const smpmgrAvailable = await bluetooth.checkSmpmgr();
  console.log(`   - smpmgr available: ${smpmgrAvailable ? '✓' : '✗'}`);
  
  const bluetoothctlAvailable = await bluetooth.checkBluetoothctl();
  console.log(`   - bluetoothctl available: ${bluetoothctlAvailable ? '✓' : '✗'}`);
  
  console.log('\n2. Running device discovery (this will take ~3 seconds)...\n');
  
  // Test scanning multiple times
  for (let i = 0; i < 3; i++) {
    console.log(`\n--- Scan ${i + 1} ---`);
    const startTime = Date.now();
    
    try {
      const devices = await bluetooth.listTempoDevices();
      const duration = Date.now() - startTime;
      
      console.log(`Scan completed in ${duration}ms`);
      console.log(`Found ${devices.length} device(s)`);
      
      if (devices.length > 0) {
        console.log('\nDevice details:');
        devices.forEach((device, index) => {
          console.log(`  ${index + 1}. Name: ${device.name}`);
          console.log(`     MAC: ${device.bluetoothId}`);
          console.log(`     RSSI: ${device.rssi} dBm`);
          console.log(`     Signal strength: ${getSignalStrength(device.rssi)}`);
        });
      }
    } catch (error) {
      console.error('Scan failed:', error);
    }
    
    if (i < 2) {
      console.log('\nWaiting 2 seconds before next scan...');
      await new Promise(resolve => setTimeout(resolve, 2000));
    }
  }
  
  console.log('\n=== Test Complete ===');
}

function getSignalStrength(rssi: number): string {
  if (rssi >= -50) return 'Excellent';
  if (rssi >= -60) return 'Good';
  if (rssi >= -70) return 'Fair';
  return 'Poor';
}

// Run the test
testBluetoothScanning().catch(console.error);