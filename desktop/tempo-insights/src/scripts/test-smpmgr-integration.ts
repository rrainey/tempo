// Save as: src/scripts/test-smpmgr-integration.ts
// Run with: tsx src/scripts/test-smpmgr-integration.ts

import { BluetoothService } from '../lib/bluetooth/bluetooth.service';
import { config } from 'dotenv';
import crypto from 'crypto';
import fs from 'fs/promises';

config({ path: '.env' });

const bluetooth = BluetoothService.getInstance();

async function testSmpmgrIntegration() {
  console.log('=== Testing smpmgr Integration ===\n');
  
  try {
    // 1. Check smpmgr availability
    console.log('1. Checking smpmgr availability:');
    const smpmgrAvailable = await bluetooth.checkSmpmgr();
    console.log(`   smpmgr available: ${smpmgrAvailable ? '✓' : '✗'}`);
    
    if (!smpmgrAvailable) {
      console.log('\n   smpmgr is not available. Please install it and set SMPMGR_PLUGIN_PATH');
      return;
    }
    
    // 2. Check bluetoothctl availability
    console.log('\n2. Checking bluetoothctl availability:');
    const bluetoothctlAvailable = await bluetooth.checkBluetoothctl();
    console.log(`   bluetoothctl available: ${bluetoothctlAvailable ? '✓' : '✗'}`);
    
    // 3. Test device discovery
    console.log('\n3. Testing device discovery:');
    console.log('   Scanning for Tempo devices (this may take 5-10 seconds)...');
    
    const devices = await bluetooth.listTempoDevices();
    console.log(`   Found ${devices.length} Tempo device(s)`);
    
    if (devices.length === 0) {
      console.log('   No devices found. Using mock mode for remaining tests.');
    } else {
      devices.forEach(device => {
        console.log(`   - ${device.name} [${device.bluetoothId}] RSSI: ${device.rssi} dBm`);
      });
    }
    
    // 4. Test session listing (if device available)
    if (devices.length > 0) {
      const testDevice = devices[0];
      console.log(`\n4. Testing session list for ${testDevice.name}:`);
      
      try {
        const sessions = await bluetooth.getSessionList(testDevice.bluetoothId);
        console.log(`   Found ${sessions.length} session(s):`);
        
        sessions.forEach((session, index) => {
          console.log(`   ${index + 1}. ${session.name} (${session.is_dir ? 'DIR' : 'FILE'}, ${session.size} bytes)`);
        });
        
        // 5. Test file listing
        console.log(`\n5. Testing file listing for ${testDevice.name}:`);
        const files = await bluetooth.listDeviceFiles(testDevice.bluetoothId);
        console.log(`   Found ${files.length} file(s):`);
        
        files.forEach((file, index) => {
          console.log(`   ${index + 1}. ${file}`);
        });
        
        // 6. Test file download (if files available)
        const jumpFiles = files.filter(f => f.endsWith('.txt') || f.endsWith('.log'));
        if (jumpFiles.length > 0) {
          console.log(`\n6. Testing file download for ${jumpFiles[0]}:`);
          
          try {
            const fileContent = await bluetooth.downloadFile(testDevice.bluetoothId, jumpFiles[0]);
            const hash = crypto.createHash('sha256').update(fileContent).digest('hex');
            
            console.log(`   ✓ Downloaded ${fileContent.length} bytes`);
            console.log(`   SHA-256: ${hash.substring(0, 32)}...`);
            
            // Show first line if it looks like flight data
            const firstLine = fileContent.toString('utf-8').split('\n')[0];
            if (firstLine.startsWith('$')) {
              console.log(`   First line: ${firstLine}`);
            }
          } catch (downloadError) {
            console.error(`   ✗ Download failed:`, downloadError);
          }
        }
        
      } catch (sessionError) {
        console.error(`   Error getting session list:`, sessionError);
      }
    }
    
    // 7. Test LED control (if device available)
    if (devices.length > 0) {
      const testDevice = devices[0];
      console.log(`\n7. Testing LED control for ${testDevice.name}:`);
      
      try {
        await bluetooth.blinkDevice(testDevice.bluetoothId, 'green');
        console.log('   ✓ LED command sent (green)');
        
        // Wait a bit
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // Turn off
        await bluetooth.blinkDevice(testDevice.bluetoothId, 'off');
        console.log('   ✓ LED command sent (LED back to default app mode)');
      } catch (ledError) {
        console.error(`   ✗ LED control failed:`, ledError);
      }
    }
    
  } catch (error) {
    console.error('Error:', error);
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testSmpmgrIntegration().catch(console.error);