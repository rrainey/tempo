// lib/analysis/event-detector.ts

import { ParsedLogData, TimeSeriesPoint } from './log-parser';

export interface JumpEvents {
  exitOffsetSec?: number;
  deploymentOffsetSec?: number;
  landingOffsetSec?: number;
  
  // Additional event metadata
  exitAltitudeFt?: number;
  deployAltitudeFt?: number;
  maxDescentRateFpm?: number;
}

export class EventDetector {
  /**
   * Detect exit from aircraft
   * Exit is defined as first sustained descent >2000 fpm for ≥1 second
   */
  static detectExit(data: ParsedLogData): { offsetSec?: number; altitudeFt?: number } {
    const { vspeed, altitude, sampleRate } = data;
    
    // Need at least 1 second of samples
    const samplesPerSecond = Math.round(sampleRate);
    const requiredSamples = samplesPerSecond;
    
    for (let i = 0; i < vspeed.length - requiredSamples; i++) {
      // Check if all samples in the next second show descent > 2000 fpm
      let allSamplesQualify = true;
      
      for (let j = 0; j < requiredSamples; j++) {
        if (vspeed[i + j].value > -2000) {
          allSamplesQualify = false;
          break;
        }
      }
      
      if (allSamplesQualify) {
        // Found exit point
        const exitTime = vspeed[i].timestamp;
        
        // Find corresponding altitude
        const altPoint = altitude.find(p => Math.abs(p.timestamp - exitTime) < (1 / sampleRate));
        
        console.log(`[EVENT DETECTOR] Exit detected at ${exitTime.toFixed(1)}s, altitude ${altPoint?.value || 'unknown'}ft`);
        
        return {
          offsetSec: exitTime,
          altitudeFt: altPoint?.value
        };
      }
    }
    
    console.log('[EVENT DETECTOR] No exit detected');
    return {};
  }
  
  /**
   * Detect deployment and activation
   * Deployment is 0.25g deceleration for 0.1s
   * Activation is first RoD <2000 fpm after deployment
   */
  static detectDeployment(data: ParsedLogData): { 
    deploymentOffsetSec?: number; 
    activationOffsetSec?: number;
    deployAltitudeFt?: number;
  } {
    const { vspeed, altitude, sampleRate } = data;
    
    // Calculate acceleration from vspeed changes
    // 0.25g = 0.25 * 32.2 ft/s² = 8.05 ft/s² = 483 ft/min²
    const gThreshold = 0.25 * 32.2 * 60; // Convert to ft/min² for fpm data
    
    // Samples needed for 0.1s
    const samplesFor100ms = Math.max(1, Math.round(sampleRate * 0.1));
    
    let deploymentTime: number | undefined;
    let deploymentAlt: number | undefined;
    
    // Look for rapid deceleration (deployment)
    for (let i = 1; i < vspeed.length - samplesFor100ms; i++) {
      // Skip if we're not in freefall (need to be going fast first)
      if (vspeed[i].value > -5000) continue;
      
      // Check deceleration over next 0.1s
      let maxDecel = 0;
      for (let j = 0; j < samplesFor100ms; j++) {
        if (i + j + 1 >= vspeed.length) break;
        
        const v1 = vspeed[i + j].value;
        const v2 = vspeed[i + j + 1].value;
        const dt = vspeed[i + j + 1].timestamp - vspeed[i + j].timestamp;
        
        if (dt > 0) {
          const decel = (v2 - v1) / (dt / 60); // Convert to per minute
          maxDecel = Math.max(maxDecel, decel);
        }
      }
      
      if (maxDecel > gThreshold) {
        deploymentTime = vspeed[i].timestamp;
        const altPoint = altitude.find(p => Math.abs(p.timestamp - deploymentTime!) < (1 / sampleRate));
        deploymentAlt = altPoint?.value;
        
        console.log(`[EVENT DETECTOR] Deployment detected at ${deploymentTime.toFixed(1)}s, altitude ${deploymentAlt || 'unknown'}ft`);
        break;
      }
    }
    
    // Look for activation (first RoD < 2000 fpm after deployment)
    let activationTime: number | undefined;
    
    if (deploymentTime !== undefined) {
      const deployIdx = vspeed.findIndex(p => p.timestamp >= deploymentTime);
      
      for (let i = deployIdx; i < vspeed.length; i++) {
        if (vspeed[i].value > -2000) {
          activationTime = vspeed[i].timestamp;
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
    const { vspeed, altitude, sampleRate } = data;
    
    // Samples needed for 10 seconds
    const samplesFor10s = Math.round(sampleRate * 10);
    
    for (let i = 0; i < vspeed.length - samplesFor10s; i++) {
      // Check if we're on or near ground first
      const currentAlt = altitude.find(p => Math.abs(p.timestamp - vspeed[i].timestamp) < (1 / sampleRate));
      if (!currentAlt || currentAlt.value > 500) continue; // Must be below 500ft
      
      // Check if all samples in next 10s show low descent rate
      let allSamplesQualify = true;
      
      for (let j = 0; j < samplesFor10s && i + j < vspeed.length; j++) {
        if (Math.abs(vspeed[i + j].value) > 100) {
          allSamplesQualify = false;
          break;
        }
      }
      
      if (allSamplesQualify) {
        const landingTime = vspeed[i].timestamp;
        console.log(`[EVENT DETECTOR] Landing detected at ${landingTime.toFixed(1)}s`);
        
        return { offsetSec: landingTime };
      }
    }
    
    // Alternative: Check if altitude reaches 0
    const groundPoint = altitude.find(p => p.value <= 0);
    if (groundPoint) {
      console.log(`[EVENT DETECTOR] Landing detected at ${groundPoint.timestamp.toFixed(1)}s (altitude reached ground)`);
      return { offsetSec: groundPoint.timestamp };
    }
    
    console.log('[EVENT DETECTOR] No landing detected');
    return {};
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
    
    // Find max descent rate during freefall
    if (events.exitOffsetSec !== undefined && events.deploymentOffsetSec !== undefined) {
      let maxDescentRate = 0;
      
      for (const point of data.vspeed) {
        if (point.timestamp >= events.exitOffsetSec && 
            point.timestamp <= events.deploymentOffsetSec) {
          maxDescentRate = Math.min(maxDescentRate, point.value);
        }
      }
      
      events.maxDescentRateFpm = Math.abs(maxDescentRate);
    }
    
    return events;
  }
}