// lib/analysis/event-detector.ts

import { ParsedLogData, TimeSeriesPoint } from './log-parser';
import { KMLDataV1, Vector3 } from './dropkick-reader';

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
      
      // Skip entries without rate of descent data
      if (entry.rateOfDescent_fpm === null) continue;
      
      // Check if this and next few entries show descent > 2000 fpm
      let sustainedDescent = true;
      let endIndex = i;
      
      // Look ahead for 1 second worth of data
      for (let j = i + 1; j < logEntries.length && j < i + 5; j++) {
        const nextEntry = logEntries[j];
        if (nextEntry.timeOffset - entry.timeOffset >= 1.0) {
          endIndex = j;
          break;
        }
        
        if (nextEntry.rateOfDescent_fpm === null || nextEntry.rateOfDescent_fpm < 2000) {
          sustainedDescent = false;
          break;
        }
      }
      
      if (sustainedDescent && endIndex > i) {
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
    
    // 0.25g = 0.25 * 9.81 m/s² = 2.45 m/s²
    const gThreshold = 0.25 * 9.81;
    
    let deploymentTime: number | undefined;
    let deploymentAlt: number | undefined;
    let peakAccel = 0;
    
    // Look for rapid deceleration using IMU data
    for (let i = 1; i < logEntries.length; i++) {
      const entry = logEntries[i];
      
      // Skip if no acceleration data or not in freefall
      if (!entry.peakAccel_mps2 || entry.rateOfDescent_fpm === null || entry.rateOfDescent_fpm < 5000) {
        continue;
      }
      
      // Check acceleration magnitude
      const accelMag = this.vectorMagnitude(entry.peakAccel_mps2);
      
      // Look for significant upward acceleration (deceleration in freefall)
      // In freefall, we expect ~1g downward, so deployment shows as increased magnitude
      if (accelMag > 9.81 + gThreshold) {
        // Additional check: vertical component should show upward acceleration
        if (entry.peakAccel_mps2.z > gThreshold) {
          deploymentTime = entry.timeOffset;
          deploymentAlt = entry.baroAlt_ft || undefined;
          peakAccel = accelMag;
          
          console.log(`[EVENT DETECTOR] Deployment detected at ${deploymentTime.toFixed(1)}s, altitude ${deploymentAlt || 'unknown'}ft, peak ${accelMag.toFixed(2)} m/s²`);
          break;
        }
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
  static detectLanding(data: ParsedLogData): { offsetSec?: number } {
    const { logEntries } = data;
    
    // Find first sustained low descent rate
    for (let i = 0; i < logEntries.length; i++) {
      const entry = logEntries[i];
      
      // Skip if no altitude data or too high
      if (entry.baroAlt_ft === null || entry.baroAlt_ft > 500) continue;
      
      // Check if descent rate is low
      if (entry.rateOfDescent_fpm !== null && Math.abs(entry.rateOfDescent_fpm) < 100) {
        // Look ahead to see if it stays low for 10 seconds
        let sustainedLowRate = true;
        let duration = 0;
        
        for (let j = i + 1; j < logEntries.length; j++) {
          const nextEntry = logEntries[j];
          duration = nextEntry.timeOffset - entry.timeOffset;
          
          if (duration >= 10) {
            break; // Found 10 seconds of data
          }
          
          if (nextEntry.rateOfDescent_fpm === null || Math.abs(nextEntry.rateOfDescent_fpm) > 100) {
            sustainedLowRate = false;
            break;
          }
        }
        
        if (sustainedLowRate && duration >= 10) {
          console.log(`[EVENT DETECTOR] Landing detected at ${entry.timeOffset.toFixed(1)}s`);
          return { offsetSec: entry.timeOffset };
        }
      }
    }
    
    // Alternative: Check final entry if at low altitude
    const lastEntry = logEntries[logEntries.length - 1];
    if (lastEntry.baroAlt_ft !== null && lastEntry.baroAlt_ft < 50) {
      console.log(`[EVENT DETECTOR] Landing detected at ${lastEntry.timeOffset.toFixed(1)}s (final altitude ${lastEntry.baroAlt_ft}ft)`);
      return { offsetSec: lastEntry.timeOffset };
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
    const landing = this.detectLanding(data);
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