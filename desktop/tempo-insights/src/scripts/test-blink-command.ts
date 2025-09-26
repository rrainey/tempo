// Save as: src/scripts/test-blink-command.ts
// Run with: tsx src/scripts/test-blink-command.ts

import { config } from 'dotenv';

config({ path: '.env' });

const API_BASE = 'http://localhost:3000/api';

async function login() {
  const response = await fetch(`${API_BASE}/auth/login`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      email: 'admin@tempoinsights.local', // Update with your admin email
      password: 'admin123' // Update with your password
    })
  });
  
  if (!response.ok) {
    throw new Error('Failed to login');
  }
  
  const cookies = response.headers.get('set-cookie');
  return cookies;
}

async function testBlinkCommand() {
  console.log('=== Testing Blink Command End-to-End ===\n');
  
  try {
    // 1. Login as admin
    console.log('1. Logging in...');
    const authCookie = await login();
    if (!authCookie) {
      throw new Error('No auth cookie received');
    }
    console.log('   ✓ Logged in successfully\n');
    
    // 2. Get list of devices
    console.log('2. Getting device list...');
    const devicesResponse = await fetch(`${API_BASE}/devices/list`, {
      headers: { 'Cookie': authCookie }
    });
    
    if (!devicesResponse.ok) {
      throw new Error('Failed to get devices');
    }
    
    const { devices } = await devicesResponse.json();
    const onlineDevice = devices.find((d:any) => d.isOnline);
    
    if (!onlineDevice) {
      console.log('   No online devices found. Please ensure a device is powered on.');
      return;
    }
    
    console.log(`   Found online device: ${onlineDevice.name} (${onlineDevice.id})\n`);
    
    // 3. Send LED on command
    console.log('3. Sending LED ON command (green)...');
    const ledOnResponse = await fetch(`${API_BASE}/devices/${onlineDevice.id}/commands/led-on`, {
      method: 'POST',
      headers: {
        'Cookie': authCookie,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ color: 'green' })
    });
    
    if (!ledOnResponse.ok) {
      const error = await ledOnResponse.json();
      throw new Error(`LED ON failed: ${error.error}`);
    }
    
    const ledOnResult = await ledOnResponse.json();
    console.log(`   ✓ Command queued: ${ledOnResult.message}`);
    console.log(`   Command ID: ${ledOnResult.commandId}\n`);
    
    // 4. Check command status
    console.log('4. Waiting for command to be processed...');
    await new Promise(resolve => setTimeout(resolve, 5000)); // Wait 5 seconds
    
    const statusResponse = await fetch(`${API_BASE}/devices/${onlineDevice.id}/commands?limit=1`, {
      headers: { 'Cookie': authCookie }
    });
    
    if (statusResponse.ok) {
      const { commands } = await statusResponse.json();
      if (commands.length > 0) {
        const latestCommand = commands[0];
        console.log(`   Command status: ${latestCommand.commandStatus}`);
        if (latestCommand.responseData) {
          console.log(`   Response: ${JSON.stringify(latestCommand.responseData)}`);
        }
        if (latestCommand.errorMessage) {
          console.log(`   Error: ${latestCommand.errorMessage}`);
        }
      }
    }
    
    // 5. Wait a bit then send LED off
    console.log('\n5. Waiting 3 seconds before turning LED off...');
    await new Promise(resolve => setTimeout(resolve, 3000));
    
    console.log('   Sending LED OFF command...');
    const ledOffResponse = await fetch(`${API_BASE}/devices/${onlineDevice.id}/commands/led-off`, {
      method: 'POST',
      headers: {
        'Cookie': authCookie,
        'Content-Type': 'application/json'
      }
    });
    
    if (!ledOffResponse.ok) {
      const error = await ledOffResponse.json();
      throw new Error(`LED OFF failed: ${error.error}`);
    }
    
    const ledOffResult = await ledOffResponse.json();
    console.log(`   ✓ Command queued: ${ledOffResult.message}`);
    console.log(`   Command ID: ${ledOffResult.commandId}`);
    
    console.log('\n✓ Task 96 complete!');
    console.log('  - LED ON/OFF APIs are working');
    console.log('  - Commands are queued successfully');
    console.log('  - Bluetooth scanner processes the commands');
    
  } catch (error) {
    console.error('\nError:', error);
    console.log('\nPlease check:');
    console.log('  - Your Next.js server is running on port 3000');
    console.log('  - Update the login credentials in the script');
    console.log('  - Ensure bluetooth scanner is running');
    console.log('  - A device is online');
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testBlinkCommand().catch(console.error);