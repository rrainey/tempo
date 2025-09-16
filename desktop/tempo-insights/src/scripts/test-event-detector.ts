// Save as: src/scripts/test-event-detector.ts
// Run with: tsx src/scripts/test-event-detector.ts

import { LogParser } from '../lib/analysis/log-parser';
import { EventDetector } from '../lib/analysis/event-detector';
import * as fs from 'fs';
import * as path from 'path';

function testEventDetector() {
  console.log('=== Testing Event Detector ===\n');
  
  // Read the sample flight data
  const sampleFilePath = path.join(process.cwd(), 'docs', 'sample-flight.txt');
  
  try {
    // Check if file exists
    if (!fs.existsSync(sampleFilePath)) {
      console.error(`Sample file not found at: ${sampleFilePath}`);
      console.error('Please ensure docs/sample-flight.txt exists');
      return;
    }
    
    // Read the file
    const fileContent = fs.readFileSync(sampleFilePath);
    console.log(`Loaded sample flight data: ${fileContent.length} bytes\n`);
    
    // Parse the log
    const validation = LogParser.validateLog(fileContent);
    if (!validation.isValid) {
      console.error(`Invalid log file: ${validation.message}`);
      return;
    }
    
    const parsedData = LogParser.parseLog(fileContent);
    
    console.log('1. Parsed jump profile summary:');
    console.log(`   - Duration: ${parsedData.duration}s`);
    console.log(`   - Data points: ${parsedData.altitude.length}`);
    console.log(`   - Sample rate: ${parsedData.sampleRate} Hz`);
    console.log(`   - Has GPS: ${parsedData.hasGPS}`);
    console.log(`   - Start time: ${parsedData.startTime.toISOString()}\n`);
    
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
    
    // Display sample data points for verification
    console.log('\n4. Sample data points from file:');
    console.log('   Time(s) | Altitude(ft) | VSpeed(fpm)');
    console.log('   --------|--------------|------------');
    
    const sampleTimes = [0, 10, 20, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360,  390, 420, 450, 480, 510, 540, 570, 600, 630, 660, 690, 720, 750, 780, 810, 840, 870, 900, 930, 960, 990, 1020, 1050, 1080, 1110, 1140, 1170, 1200];
    sampleTimes.forEach(t => {
      const altPoint = parsedData.altitude.find(p => Math.abs(p.timestamp - t) < 0.5);
      const vsPoint = parsedData.vspeed.find(p => Math.abs(p.timestamp - t) < 0.5);
      
      if (altPoint && vsPoint) {
        console.log(`   ${t.toString().padStart(7)} | ${altPoint.value.toString().padStart(12)} | ${vsPoint.value.toString().padStart(11)}`);
      }
    });
    
    // Show GPS data if available
    if (parsedData.hasGPS && parsedData.gps.length > 0) {
      console.log('\n5. GPS data from file:');
      console.log('   Found GPS data with', parsedData.gps.length, 'points');
      const firstGPS = parsedData.gps[0];
      const lastGPS = parsedData.gps[parsedData.gps.length - 1];
      console.log(`   - First GPS: ${firstGPS.latitude.toFixed(6)}, ${firstGPS.longitude.toFixed(6)} at ${firstGPS.timestamp}s`);
      console.log(`   - Last GPS: ${lastGPS.latitude.toFixed(6)}, ${lastGPS.longitude.toFixed(6)} at ${lastGPS.timestamp}s`);
    }
    
  } catch (error) {
    console.error('Error reading sample file:', error);
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testEventDetector();