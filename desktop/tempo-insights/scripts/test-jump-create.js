const crypto = require('crypto');

async function testCreateJumpLog() {
  // Replace with your actual values
  const API_URL = 'http://localhost:3000/api/internal/jumps/create';
  const INTERNAL_TOKEN = 'your-super-secret-internal-token-change-this';

  // Create mock jump log data
  const mockLogData = Buffer.from('Mock jump log data for testing ' + Date.now());
  const mockLogBase64 = mockLogData.toString('base64');

  // You'll need to get these IDs from your database
  // For now, we'll use placeholders
  const testData = {
    deviceId: 'cmfa8c48t0004fg4wpyieta9j', // Replace with actual device ID from DB
    userId: 'cmf9v4qzo0000fgx063jvwzgy',     // Replace with actual user ID from DB
    rawLogBase64: mockLogBase64,
    fileName: `test_jump_${Date.now()}.dat`,
    timestamp: new Date().toISOString(),
  };

  try {
    const response = await fetch(API_URL, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'x-internal-token': INTERNAL_TOKEN,
      },
      body: JSON.stringify(testData),
    });

    const result = await response.json();

    if (response.ok) {
      console.log('Success:', result);
      console.log('Jump log hash:', result.jumpLog?.hash);
    } else {
      console.error('Error:', result);
    }
  } catch (error) {
    console.error('Request failed:', error);
  }
}

// Get device and user IDs first
console.log('Before running this test:');
console.log('1. Get a device ID from your database');
console.log('2. Get a user ID from your database');
console.log('3. Update the INTERNAL_TOKEN if you changed it');
console.log('4. Update testData with actual IDs');
console.log('\nThen uncomment the line below:');
testCreateJumpLog();
