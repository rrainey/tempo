// Save as: src/scripts/test-parser.ts
// Run with: tsx src/scripts/test-parser.ts

import { LogParser } from '../lib/analysis/log-processor';
function testLogParser() {
  console.log('=== Testing Log Parser ===\n');
  
  // Test 1: Parse mock jump data
  console.log('1. Testing parser with mock jump data:');
  const mockLog = Buffer.from('MOCK_JUMP_DATA_FOR_TESTING\nThis is a test log file');
  
  // Validate first
  const validation = LogParser.validateLog(mockLog);
  console.log(`   Validation: ${validation.isValid ? '✓ Valid' : '✗ Invalid'}`);
  if (validation.message) {
    console.log(`   Message: ${validation.message}`);
  }
  
  // Parse the log
  console.log('\n   Parsing log...');
  const parsed = LogParser.parseLog(mockLog);
  
  console.log(`\n   Parsed data summary:`);
  console.log(`   - Start time: ${parsed.startTime.toISOString()}`);
  console.log(`   - Duration: ${parsed.duration} seconds`);
  console.log(`   - Sample rate: ${parsed.sampleRate} Hz`);
  console.log(`   - Altitude points: ${parsed.altitude.length}`);
  console.log(`   - VSpeed points: ${parsed.vspeed.length}`);
  console.log(`   - GPS points: ${parsed.gps.length}`);
  console.log(`   - Has GPS: ${parsed.hasGPS ? 'Yes' : 'No'}`);
  
  // Show sample of altitude profile
  console.log('\n2. Altitude profile sample:');
  const altitudeSamples = [0, 10, 20, 25, 40, 60, 80, 85, 100, 120, 180];
  console.log('   Time(s) | Altitude(ft) | VSpeed(fpm)');
  console.log('   --------|--------------|------------');
  
  altitudeSamples.forEach(t => {
    const altPoint = parsed.altitude.find(p => Math.abs(p.timestamp - t) < 0.1);
    const vsPoint = parsed.vspeed.find(p => Math.abs(p.timestamp - t) < 0.1);
    
    if (altPoint && vsPoint) {
      console.log(`   ${t.toString().padStart(7)} | ${altPoint.value.toString().padStart(12)} | ${vsPoint.value.toString().padStart(11)}`);
    }
  });
  
  // Identify key events
  console.log('\n3. Detected events (based on mock data):');
  
  // Find exit (first big negative vspeed)
  const exitIndex = parsed.vspeed.findIndex(p => p.value < -2000);
  if (exitIndex >= 0) {
    const exitTime = parsed.vspeed[exitIndex].timestamp;
    const exitAlt = parsed.altitude.find(p => Math.abs(p.timestamp - exitTime) < 0.1);
    console.log(`   - Exit detected at ${exitTime.toFixed(1)}s, altitude ${exitAlt?.value || 'unknown'}ft`);
  }
  
  // Find deployment (vspeed goes from very negative to less negative)
  let deployIndex = -1;
  for (let i = 1; i < parsed.vspeed.length; i++) {
    if (parsed.vspeed[i-1].value < -8000 && parsed.vspeed[i].value > -4000) {
      deployIndex = i;
      break;
    }
  }
  
  if (deployIndex >= 0) {
    const deployTime = parsed.vspeed[deployIndex].timestamp;
    const deployAlt = parsed.altitude.find(p => Math.abs(p.timestamp - deployTime) < 0.1);
    console.log(`   - Deployment detected at ${deployTime.toFixed(1)}s, altitude ${deployAlt?.value || 'unknown'}ft`);
  }
  
  // Find landing (altitude reaches 0)
  const landingPoint = parsed.altitude.find(p => p.value === 0);
  if (landingPoint) {
    console.log(`   - Landing detected at ${landingPoint.timestamp.toFixed(1)}s`);
  }
  
  // Test 2: GPS data
  if (parsed.hasGPS && parsed.gps.length > 0) {
    console.log('\n4. GPS data sample:');
    console.log('   Time(s) | Latitude    | Longitude   | Alt(ft)');
    console.log('   --------|-------------|-------------|--------');
    
    const gpsSamples = [0, 30, 60, 90, 120, 180];
    gpsSamples.forEach(t => {
      const gpsPoint = parsed.gps.find(p => Math.abs(p.timestamp - t) < 0.5);
      if (gpsPoint) {
        console.log(`   ${t.toString().padStart(7)} | ${gpsPoint.latitude.toFixed(6).padStart(11)} | ${gpsPoint.longitude.toFixed(6).padStart(11)} | ${gpsPoint.altitude.toString().padStart(7)}`);
      }
    });
  }
  
  // Test 3: Validation edge cases
  console.log('\n5. Testing validation edge cases:');
  
  const testCases = [
    { name: 'Empty buffer', data: Buffer.from('') },
    { name: 'Too small', data: Buffer.from('short') },
    { name: 'Large buffer', data: Buffer.alloc(1024 * 1024) }, // 1MB
  ];
  
  testCases.forEach(test => {
    const result = LogParser.validateLog(test.data);
    console.log(`   - ${test.name}: ${result.isValid ? '✓' : '✗'} ${result.message || ''}`);
  });
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testLogParser();