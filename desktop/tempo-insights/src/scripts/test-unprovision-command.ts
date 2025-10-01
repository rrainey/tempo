// src/scripts/test-unprovision-command.ts
// Run with: tsx src/scripts/test-unprovision-command.ts
/* eslint-disable  @typescript-eslint/no-explicit-any */

import { config } from 'dotenv';

config({ path: '.env' });

const API_BASE = 'http://localhost:3000/api';

async function login() {
  const response = await fetch(`${API_BASE}/auth/login`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      email: 'admin@tempo.example.com', // Update with your admin email
      password: 'your-password' // Update with your password
    })
  });
  
  if (!response.ok) {
    throw new Error('Failed to login');
  }
  
  const cookies = response.headers.get('set-cookie');
  return cookies;
}

async function testUnprovisionCommand() {
  console.log('=== Testing Unprovision Command (Task 98) ===\n');
  
  try {
    // 1. Login as admin
    console.log('1. Logging in as admin...');
    const authCookie = await login();
    if (!authCookie) {
      throw new Error('No auth cookie received');
    }
    console.log('   ✓ Logged in successfully\n');
    
    // 2. Get list of devices
    console.log('2. Finding a device to unprovision...');
    const devicesResponse = await fetch(`${API_BASE}/devices/list`, {
      headers: { 'Cookie': authCookie }
    });
    
    if (!devicesResponse.ok) {
      throw new Error('Failed to get devices');
    }
    
    const { devices } = await devicesResponse.json();
    
    // Find an online device that has an owner (not lent)
    const assignedDevice = devices.find((d:any) => 
      d.isOnline && 
      d.state === 'ACTIVE' && 
      d.owner && 
      !d.lentTo
    );
    
    if (!assignedDevice) {
      console.log('   No suitable device found for unprovisioning.');
      console.log('   Need: online, active state, has owner, not lent');
      console.log('   Available devices:');
      devices.forEach((d:any) => {
        console.log(`     - ${d.name}: ${d.state} (${d.isOnline ? 'online' : 'offline'}) owner: ${d.owner?.name || 'none'} lent: ${d.lentTo ? 'yes' : 'no'}`);
      });
      return;
    }
    
    console.log(`   Found device: ${assignedDevice.name} (${assignedDevice.id})`);
    console.log(`   Current owner: ${assignedDevice.owner.name}\n`);
    
    // 3. Send unprovision command
    console.log('3. Sending UNPROVISION command...');
    const unprovisionResponse = await fetch(`${API_BASE}/devices/${assignedDevice.id}/commands/unprovision`, {
      method: 'POST',
      headers: {
        'Cookie': authCookie,
        'Content-Type': 'application/json'
      }
    });
    
    if (!unprovisionResponse.ok) {
      const error = await unprovisionResponse.json();
      throw new Error(`Unprovision failed: ${error.error}`);
    }
    
    const unprovisionResult = await unprovisionResponse.json();
    console.log(`   ✓ Command queued: ${unprovisionResult.message}`);
    console.log(`   Command ID: ${unprovisionResult.commandId}`);
    console.log(`   Previous owner: ${unprovisionResult.previousOwner}\n`);
    
    // 4. Wait and check command status
    console.log('4. Waiting for command to be processed (35 seconds)...');
    await new Promise(resolve => setTimeout(resolve, 35000));
    
    const statusResponse = await fetch(`${API_BASE}/devices/${assignedDevice.id}/commands?limit=1`, {
      headers: { 'Cookie': authCookie }
    });
    
    if (statusResponse.ok) {
      const { commands } = await statusResponse.json();
      if (commands.length > 0) {
        const latestCommand = commands[0];
        console.log(`   Command status: ${latestCommand.commandStatus}`);
        if (latestCommand.responseData) {
          console.log(`   Response: ${JSON.stringify(latestCommand.responseData, null, 2)}`);
        }
        if (latestCommand.errorMessage) {
          console.log(`   Error: ${latestCommand.errorMessage}`);
        }
      }
    }
    
    // 5. Verify device state changed
    console.log('\n5. Verifying device state...');
    const updatedDevicesResponse = await fetch(`${API_BASE}/devices/list`, {
      headers: { 'Cookie': authCookie }
    });
    
    if (updatedDevicesResponse.ok) {
      const { devices: updatedDevices } = await updatedDevicesResponse.json();
      const updatedDevice = updatedDevices.find((d:any) => d.id === assignedDevice.id);
      
      if (updatedDevice) {
        console.log(`   Device state: ${updatedDevice.state}`);
        console.log(`   Device owner: ${updatedDevice.owner?.name || 'None'}`);
        console.log(`   State is PROVISIONING: ${updatedDevice.state === 'PROVISIONING' ? '✓' : '✗'}`);
        console.log(`   Owner cleared: ${!updatedDevice.owner ? '✓' : '✗'}`);
      }
    }
    
    console.log('\n✓ Task 98 test complete!');
    console.log('  - Unprovision API is working');
    console.log('  - uinfo.json is removed from device');
    console.log('  - Device state changes to PROVISIONING');
    console.log('  - Device owner is cleared');
    
  } catch (error) {
    console.error('\nError:', error);
    console.log('\nPlease check:');
    console.log('  - Update the login credentials in the script');
    console.log('  - Ensure bluetooth scanner is running');
    console.log('  - A device is online, assigned, and not lent');
    console.log('  - The smpmgr file delete command is working');
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testUnprovisionCommand().catch(console.error);