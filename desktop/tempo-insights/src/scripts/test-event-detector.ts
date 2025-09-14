// Save as: src/scripts/test-event-detector.ts
// Run with: tsx src/scripts/test-event-detector.ts

import { LogParser } from '../lib/analysis/log-parser';
import { EventDetector } from '../lib/analysis/event-detector';

function testEventDetector() {
  console.log('=== Testing Event Detector ===\n');
  
  // Generate a mock jump log
  const mockLog = Buffer.from('MOCK_JUMP_DATA_FOR_TESTING');
  const parsedData = LogParser.parseLog(mockLog);
  
  console.log('1. Generated jump profile summary:');
  console.log(`   - Duration: ${parsedData.duration}s`);
  console.log(`   - Data points: ${parsedData.altitude.length}`);
  console.log(`   - Sample rate: ${parsedData.sampleRate} Hz`);
  console.log(`   - Has GPS: ${parsedData.hasGPS}\n`);
  
  // Test individual event detection
  console.log('2. Testing individual event detection:\n');
  
  // Test exit detection
  console.log('   Exit Detection:');
  const exit = EventDetector.detectExit(parsedData);
  if (exit.offsetSec !== undefined) {
    console.log(`   ✓ Exit detected at ${exit.offsetSec.toFixed(1)}s`);
    console.log(`   - Altitude: ${exit.altitudeFt}ft`);
    
    // Verify exit parameters
    const exitIdx = parsedData.vspeed.findIndex(p => p.timestamp >= exit.offsetSec!);
    if (exitIdx >= 0) {
      console.log(`   - Descent rate at exit: ${parsedData.vspeed[exitIdx].value} fpm`);
    }
  } else {
    console.log('   ✗ No exit detected');
  }
  
  // Test deployment detection
  console.log('\n   Deployment Detection:');
  const deployment = EventDetector.detectDeployment(parsedData);
  if (deployment.deploymentOffsetSec !== undefined) {
    console.log(`   ✓ Deployment detected at ${deployment.deploymentOffsetSec.toFixed(1)}s`);
    console.log(`   - Altitude: ${deployment.deployAltitudeFt}ft`);
    
    if (deployment.activationOffsetSec !== undefined) {
      console.log(`   ✓ Activation detected at ${deployment.activationOffsetSec.toFixed(1)}s`);
      const activationDelay = deployment.activationOffsetSec - deployment.deploymentOffsetSec;
      console.log(`   - Activation delay: ${activationDelay.toFixed(1)}s`);
    }
  } else {
    console.log('   ✗ No deployment detected');
  }
  
  // Test landing detection  
  console.log('\n   Landing Detection:');
  const landing = EventDetector.detectLanding(parsedData);
  if (landing.offsetSec !== undefined) {
    console.log(`   ✓ Landing detected at ${landing.offsetSec.toFixed(1)}s`);
    
    // Find altitude at landing
    const landingAlt = parsedData.altitude.find(p => 
      Math.abs(p.timestamp - landing.offsetSec!) < (1 / parsedData.sampleRate)
    );
    console.log(`   - Altitude: ${landingAlt?.value || 0}ft`);
  } else {
    console.log('   ✗ No landing detected');
  }
  
  // Test full jump analysis
  console.log('\n3. Full jump analysis:');
  const events = EventDetector.analyzeJump(parsedData);
  
  console.log('\n   Detected Events:');
  console.log(`   - Exit: ${events.exitOffsetSec?.toFixed(1) || 'Not detected'}s`);
  console.log(`   - Deployment: ${events.deploymentOffsetSec?.toFixed(1) || 'Not detected'}s`);
  console.log(`   - Landing: ${events.landingOffsetSec?.toFixed(1) || 'Not detected'}s`);
  
  if (events.exitOffsetSec !== undefined && events.deploymentOffsetSec !== undefined) {
    const freefallTime = events.deploymentOffsetSec - events.exitOffsetSec;
    console.log(`\n   Calculated Metrics:`);
    console.log(`   - Freefall time: ${freefallTime.toFixed(1)}s`);
    console.log(`   - Exit altitude: ${events.exitAltitudeFt || 'Unknown'}ft`);
    console.log(`   - Deploy altitude: ${events.deployAltitudeFt || 'Unknown'}ft`);
    
    if (events.exitAltitudeFt && events.deployAltitudeFt) {
      const altLoss = events.exitAltitudeFt - events.deployAltitudeFt;
      const avgFallRate = (altLoss / freefallTime) * 60; // fps to fpm
      console.log(`   - Altitude lost: ${altLoss}ft`);
      console.log(`   - Avg fall rate: ${Math.round(avgFallRate)} fpm (${Math.round(avgFallRate / 88)} mph)`);
    }
    
    if (events.maxDescentRateFpm) {
      console.log(`   - Max fall rate: ${Math.round(events.maxDescentRateFpm)} fpm (${Math.round(events.maxDescentRateFpm / 88)} mph)`);
    }
  }
  
  if (events.deploymentOffsetSec !== undefined && events.landingOffsetSec !== undefined) {
    const canopyTime = events.landingOffsetSec - events.deploymentOffsetSec;
    console.log(`   - Canopy time: ${canopyTime.toFixed(1)}s`);
  }
  
  // Test with edge cases
  console.log('\n4. Testing edge cases:');
  
  // Create data with no clear exit
  const noExitData = {
    ...parsedData,
    vspeed: parsedData.vspeed.map(p => ({ ...p, value: Math.max(p.value, -1500) }))
  };
  
  const noExitEvents = EventDetector.analyzeJump(noExitData);
  console.log(`   - No exit scenario: Exit detected = ${noExitEvents.exitOffsetSec !== undefined}`);
  
  // Create data with no landing
  const noLandingData = {
    ...parsedData,
    altitude: parsedData.altitude.map(p => ({ ...p, value: Math.max(p.value, 1000) })),
    vspeed: parsedData.vspeed.map(p => ({ ...p, value: p.value < -100 ? p.value : -500 }))
  };
  
  const noLandingEvents = EventDetector.analyzeJump(noLandingData);
  console.log(`   - No landing scenario: Landing detected = ${noLandingEvents.landingOffsetSec !== undefined}`);
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testEventDetector();