// lib/analysis/log-parser.ts

export interface TimeSeriesPoint {
  timestamp: number; // Seconds from log start
  value: number;
}

export interface GPSPoint {
  timestamp: number; // Seconds from log start
  latitude: number;
  longitude: number;
  altitude: number; // Feet
}

export interface ParsedLogData {
  // Metadata
  startTime: Date;
  duration: number; // Total log duration in seconds
  sampleRate: number; // Hz
  
  // Time series data
  altitude: TimeSeriesPoint[]; // Altitude in feet
  vspeed: TimeSeriesPoint[]; // Vertical speed in fpm (feet per minute)
  gps: GPSPoint[]; // GPS positions if available
  
  // Flags
  hasGPS: boolean;
  hasValidData: boolean;
  errorMessage?: string;
}

export class LogParser {
  /**
   * Parse a raw jump log and extract time series data
   * @param rawLog - Raw log data as Buffer
   * @returns Parsed time series data
   */
  static parseLog(rawLog: Buffer): ParsedLogData {
    // Mock implementation for now
    // Real implementation would parse the actual log format
    
    const logString = rawLog.toString('utf-8');
    const logSize = rawLog.length;
    
    console.log(`[PARSER] Parsing log of ${logSize} bytes`);
    
    // Generate mock data for testing
    const startTime = new Date();
    const duration = 180; // 3 minute jump
    const sampleRate = 4; // 4 Hz
    
    // Generate altitude series (typical jump profile)
    const altitude: TimeSeriesPoint[] = [];
    const vspeed: TimeSeriesPoint[] = [];
    
    for (let t = 0; t <= duration; t += 1/sampleRate) {
      let alt: number;
      let vs: number;
      
      if (t < 10) {
        // Climb phase (in aircraft)
        alt = 3000 + t * 100;
        vs = 600; // 600 fpm climb
      } else if (t < 20) {
        // Level flight at altitude
        alt = 14000;
        vs = 0;
      } else if (t < 25) {
        // Exit and initial freefall
        alt = 14000 - (t - 20) * 200 * 60; // Accelerating from 0
        vs = -(t - 20) * 2000; // Building up speed
      } else if (t < 80) {
        // Stable freefall at ~120 mph (10,560 fpm)
        alt = Math.max(3500, 14000 - (t - 20) * 176); // 176 fps = ~120 mph
        vs = -10560; // 120 mph = 176 fps = 10,560 fpm
      } else if (t < 85) {
        // Deployment (deceleration)
        alt = 3500 - (t - 80) * 50;
        vs = -3000 + (t - 80) * 400; // Slowing down
      } else {
        // Canopy flight
        alt = Math.max(0, 3000 - (t - 85) * 15); // ~900 fpm descent
        vs = alt > 0 ? -900 : 0;
      }
      
      altitude.push({ timestamp: t, value: Math.round(alt) });
      vspeed.push({ timestamp: t, value: Math.round(vs) });
    }
    
    // Generate GPS data (if available)
    const hasGPS = Math.random() > 0.3; // 70% chance of GPS
    const gps: GPSPoint[] = [];
    
    if (hasGPS) {
      // Mock GPS track around a drop zone
      const baseLat = 28.1234; // Example: Florida DZ
      const baseLon = -81.5678;
      
      for (let t = 0; t <= duration; t += 1) { // GPS at 1 Hz
        const altPoint = altitude.find(p => Math.abs(p.timestamp - t) < 0.1);
        if (!altPoint) continue;
        
        // Simple circular pattern for jump run and landing pattern
        const angle = (t / duration) * Math.PI * 2;
        const radius = 0.01; // About 1km radius
        
        gps.push({
          timestamp: t,
          latitude: baseLat + Math.sin(angle) * radius,
          longitude: baseLon + Math.cos(angle) * radius,
          altitude: altPoint.value
        });
      }
    }
    
    return {
      startTime,
      duration,
      sampleRate,
      altitude,
      vspeed,
      gps,
      hasGPS,
      hasValidData: true,
      errorMessage: undefined
    };
  }
  
  /**
   * Validate if the raw log appears to be valid jump data
   */
  static validateLog(rawLog: Buffer): { isValid: boolean; message?: string } {
    if (!rawLog || rawLog.length === 0) {
      return { isValid: false, message: 'Empty log file' };
    }
    
    if (rawLog.length < 100) {
      return { isValid: false, message: 'Log file too small' };
    }
    
    if (rawLog.length > 16 * 1024 * 1024) {
      return { isValid: false, message: 'Log file too large (>16MB)' };
    }
    
    // Check for common log format markers (mock for now)
    const header = rawLog.toString('utf-8', 0, Math.min(100, rawLog.length));
    if (header.includes('MOCK_JUMP_DATA')) {
      return { isValid: true };
    }
    
    // For real logs, we would check for specific format headers
    // For now, accept anything that's not obviously wrong
    return { isValid: true };
  }
}