// lib/analysis/event-detector.ts

import { ParsedLogData, TimeSeriesPoint } from './log-parser';
import { KMLDataV1, Vector3 } from './dropkick-reader';
import { METERStoFEET } from './dropkick-tools';

export interface JumpEvents {
  exitOffsetSec?: number;
  deploymentOffsetSec?: number;
  landingOffsetSec?: number;
  
  // Additional event metadata
  exitAltitudeFt?: number;
  deployAltitudeFt?: number;
  maxDescentRateFpm?: number;
  
  // New fields from DropkickReader data
  peakAcceleration?: number; // m/s²
  exitLatitude?: number;
  exitLongitude?: number;
}

export class EventDetector {
  /**
   * Detect exit from aircraft
   * Exit is defined as first sustained descent >2000 fpm for ≥1 second
   */
  static detectExit(data: ParsedLogData): { offsetSec?: number; altitudeFt?: number; latitude?: number; longitude?: number } {
    const { vspeed, logEntries } = data;
    
    // We can work directly with logEntries which have better time correlation
    for (let i = 0; i < logEntries.length - 4; i++) { // Need at least 4 samples for ~1 second
      const entry = logEntries[i];
      
      // Skip if no acceleration data or not in freefall
      if (!entry.accel_mps2 || entry.rateOfDescent_fpm === null || entry.rateOfDescent_fpm < 5000) {
        continue;
      }

      const accelMag = Math.sqrt(entry.accel_mps2.x * entry.accel_mps2.x +
                            entry.accel_mps2.y * entry.accel_mps2.y +
                            entry.accel_mps2.z * entry.accel_mps2.z);
      
      if (accelMag < 9.81 * 0.8) {
        // Found exit point
        console.log(`[EVENT DETECTOR] Exit detected at ${entry.timeOffset.toFixed(1)}s, altitude ${entry.baroAlt_ft || 'unknown'}ft`);
        
        return {
          offsetSec: entry.timeOffset,
          altitudeFt: entry.baroAlt_ft || undefined,
          latitude: entry.location?.lat_deg,
          longitude: entry.location?.lon_deg
        };
      }
    }
    
    console.log('[EVENT DETECTOR] No exit detected');
    return {};
  }
  
  /**
   * Detect deployment using acceleration data from IMU
   * Deployment is 0.25g deceleration for 0.1s
   */
  static detectDeployment(data: ParsedLogData): { 
    deploymentOffsetSec?: number; 
    activationOffsetSec?: number;
    deployAltitudeFt?: number;
  } {
    const { logEntries } = data;
    
    const gThreshold = 1.5 * 9.81;
    
    let deploymentTime: number | undefined;
    let deploymentAlt: number | undefined;
    let peakAccel = 0;
    
    // Look for rapid deceleration using IMU data
    for (let i = 1; i < logEntries.length; i++) {
      const entry = logEntries[i];
      
      // Skip if no acceleration data or not in freefall
      if (!entry.accel_mps2 || entry.rateOfDescent_fpm === null || entry.rateOfDescent_fpm < 5000) {
        continue;
      }

      const accelMag = Math.sqrt(entry.accel_mps2.x * entry.accel_mps2.x +
                            entry.accel_mps2.y * entry.accel_mps2.y +
                            entry.accel_mps2.z * entry.accel_mps2.z);

      // Look for significant acceleration

      const threshold_mps2 = 9.81 + gThreshold;

      if (accelMag > threshold_mps2) {

          deploymentTime = entry.timeOffset;
          deploymentAlt = entry.baroAlt_ft || undefined;
          peakAccel = accelMag;
          
          console.log(`[EVENT DETECTOR] Deployment detected at ${deploymentTime.toFixed(1)}s, altitude ${deploymentAlt || 'unknown'}ft, peak ${accelMag.toFixed(2)} m/s²`);
          break;
      }
    }
    
    // Look for activation (first RoD < 2000 fpm after deployment)
    let activationTime: number | undefined;
    
    if (deploymentTime !== undefined) {
      const deployIdx = logEntries.findIndex(e => e.timeOffset >= deploymentTime);
      
      for (let i = deployIdx; i < logEntries.length; i++) {
        const entry = logEntries[i];
        if (entry.rateOfDescent_fpm !== null && entry.rateOfDescent_fpm < 2000) {
          activationTime = entry.timeOffset;
          console.log(`[EVENT DETECTOR] Activation detected at ${activationTime.toFixed(1)}s`);
          break;
        }
      }
    }
    
    return {
      deploymentOffsetSec: deploymentTime,
      activationOffsetSec: activationTime,
      deployAltitudeFt: deploymentAlt
    };
  }
  
  /**
   * Detect landing
   * Landing is RoD <100 fpm for 10s
   */
  static detectLanding(data: ParsedLogData, deploymentOffset_sec: number  ): { offsetSec?: number } {
    const { logEntries } = data;

    if (data.dzSurfacePressureAltitude_m === undefined) {
        console.log('[EVENT DETECTOR] No DZ surface altitude available, cannot detect landing');
        return {};
    }

    const dzSurfaceGPSAltitude_ft = METERStoFEET(data.dzSurfacePressureAltitude_m);
    
    // Find first sustained low descent rate
    for (let i = 0; i < logEntries.length; i++) {
      const entry = logEntries[i];

      if (entry.timeOffset < deploymentOffset_sec) continue;

      // Skip if no altitude data 
      if (entry.baroAlt_ft === null ) continue;

      const diff = Math.abs(entry.baroAlt_ft - dzSurfaceGPSAltitude_ft);

      // probably too high to call that a landing
      if (diff > 100) continue; 
      
      // Look ahead to see if it stays low for 10 seconds
      let duration = 0;
      
      for (let j = i + 1; j < logEntries.length; j++) {
        const nextEntry = logEntries[j];
        duration = nextEntry.timeOffset - entry.timeOffset;

        if (nextEntry.baroAlt_ft === null) continue;

        const diff = Math.abs(nextEntry.baroAlt_ft - entry.baroAlt_ft);

        //console.log(`[EVENT DETECTOR] Landing check at ${entry.timeOffset.toFixed(1)}s, altitude ${entry.baroAlt_ft}ft, diff ${diff.toFixed(1)}ft, duration ${duration.toFixed(1)}s`);  

        if (diff > 20.0) {
            continue; // too much altitude change to pass the test
        }
        
        if (duration >= 20) {
          console.log(`[EVENT DETECTOR] Landing detected at ${entry.timeOffset.toFixed(1)}s`);
          return { offsetSec: entry.timeOffset };
        }
        
      }

    }
  
    console.log('[EVENT DETECTOR] No landing detected');
    return {};
  }
  
  /**
   * Calculate magnitude of a 3D vector
   */
  private static vectorMagnitude(v: Vector3): number {
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }
  
  /**
   * Analyze all events in a jump
   */
  static analyzeJump(data: ParsedLogData): JumpEvents {
    const events: JumpEvents = {};
    
    // Detect exit
    const exit = this.detectExit(data);
    if (exit.offsetSec !== undefined) {
      events.exitOffsetSec = exit.offsetSec;
      events.exitAltitudeFt = exit.altitudeFt;
      events.exitLatitude = exit.latitude;
      events.exitLongitude = exit.longitude;
    }
    
    // Detect deployment
    const deployment = this.detectDeployment(data);
    if (deployment.deploymentOffsetSec !== undefined) {
      events.deploymentOffsetSec = deployment.deploymentOffsetSec;
      events.deployAltitudeFt = deployment.deployAltitudeFt;
    }
    
    // Detect landing
    const landing = this.detectLanding(data, events.deploymentOffsetSec || 30.0);
    if (landing.offsetSec !== undefined) {
      events.landingOffsetSec = landing.offsetSec;
    }
    
    // Find max descent rate and peak acceleration during freefall
    if (events.exitOffsetSec !== undefined && events.deploymentOffsetSec !== undefined) {
      let maxDescentRate = 0;
      let peakAccel = 0;
      
      for (const entry of data.logEntries) {
        if (entry.timeOffset >= events.exitOffsetSec && 
            entry.timeOffset <= events.deploymentOffsetSec) {
          
          // Track max descent rate
          if (entry.rateOfDescent_fpm !== null) {
            maxDescentRate = Math.max(maxDescentRate, entry.rateOfDescent_fpm);
          }
          
          // Track peak acceleration
          if (entry.peakAccel_mps2) {
            const mag = this.vectorMagnitude(entry.peakAccel_mps2);
            peakAccel = Math.max(peakAccel, mag);
          }
        }
      }
      
      events.maxDescentRateFpm = maxDescentRate;
      events.peakAcceleration = peakAccel;
    }
    
    return events;
  }
}