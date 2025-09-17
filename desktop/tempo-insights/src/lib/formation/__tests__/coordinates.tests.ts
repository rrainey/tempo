// lib/formation/__tests__/coordinates.test.ts
import {
  wgs84ToNEDDZ,
  nedDZToBaseExitFrame,
  calibrateFallRate,
  interpolatePosition,
  projectFormationAtTime
} from '../coordinates';
import { Vector3, GeodeticCoordinates } from '../types';

describe('Coordinate Transformations', () => {
  const dzCenter: GeodeticCoordinates = {
    lat_deg: 33.6320,
    lon_deg: -117.2510,
    alt_m: 436.5
  };

  describe('wgs84ToNEDDZ', () => {
    it('should return zero vector for same position', () => {
      const result = wgs84ToNEDDZ(dzCenter, dzCenter);
      expect(result.x).toBeCloseTo(0, 5);
      expect(result.y).toBeCloseTo(0, 5);
      expect(result.z).toBeCloseTo(0, 5);
    });

    it('should correctly transform north offset', () => {
      const northPoint = {
        lat_deg: dzCenter.lat_deg + 0.001, // ~111m north
        lon_deg: dzCenter.lon_deg,
        alt_m: dzCenter.alt_m
      };
      const result = wgs84ToNEDDZ(northPoint, dzCenter);
      expect(result.x).toBeCloseTo(111, 0); // North
      expect(result.y).toBeCloseTo(0, 1); // East
      expect(result.z).toBeCloseTo(0, 5); // Down
    });

    it('should correctly handle altitude differences', () => {
      const higherPoint = {
        lat_deg: dzCenter.lat_deg,
        lon_deg: dzCenter.lon_deg,
        alt_m: dzCenter.alt_m + 1000
      };
      const result = wgs84ToNEDDZ(higherPoint, dzCenter);
      expect(result.z).toBeCloseTo(-1000, 5); // Negative down = up
    });
  });

  describe('nedDZToBaseExitFrame', () => {
    it('should handle zero rotation', () => {
      const nedPos = { x: 10, y: 20, z: 30 };
      const basePos = { x: 0, y: 0, z: 0 };
      const result = nedDZToBaseExitFrame(nedPos, basePos, 0);
      expect(result).toEqual(nedPos);
    });

    it('should correctly rotate 90 degrees', () => {
      const nedPos = { x: 10, y: 0, z: 0 };
      const basePos = { x: 0, y: 0, z: 0 };
      const result = nedDZToBaseExitFrame(nedPos, basePos, 90);
      expect(result.x).toBeCloseTo(0, 5);
      expect(result.y).toBeCloseTo(10, 5);
      expect(result.z).toBeCloseTo(0, 5);
    });

    it('should translate to base position', () => {
      const nedPos = { x: 100, y: 100, z: 100 };
      const basePos = { x: 50, y: 50, z: 50 };
      const result = nedDZToBaseExitFrame(nedPos, basePos, 0);
      expect(result.x).toBeCloseTo(50, 5);
      expect(result.y).toBeCloseTo(50, 5);
      expect(result.z).toBeCloseTo(50, 5);
    });
  });

  describe('calibrateFallRate', () => {
    it('should return reference rate at 7000ft', () => {
      const verticalSpeed_mps = -53.6448; // 120 mph
      const result = calibrateFallRate(verticalSpeed_mps, 7000);
      expect(result).toBeCloseTo(120, 1);
    });

    it('should apply correct factor at 14000ft', () => {
      const verticalSpeed_mps = -60.0403; // ~134.3 mph uncalibrated
      const result = calibrateFallRate(verticalSpeed_mps, 14000);
      // 134.3 mph / 0.8955 = ~150 mph normalized
      expect(result).toBeCloseTo(150, 0);
    });

    it('should interpolate between table values', () => {
      const verticalSpeed_mps = -53.6448; // 120 mph
      const result = calibrateFallRate(verticalSpeed_mps, 11000);
      // Should be between factors for 10k and 12k feet
      expect(result).toBeGreaterThan(120 / 0.9545); // 10k factor
      expect(result).toBeLessThan(120 / 0.9247); // 12k factor
    });
  });

  describe('interpolatePosition', () => {
    const timeSeries = [
      {
        timeOffset: 0,
        location: { lat_deg: 33.6320, lon_deg: -117.2510, alt_m: 4000 },
        baroAlt_ft: 13123,
        groundtrack_degT: 180,
        groundspeed_kmph: 20
      },
      {
        timeOffset: 1,
        location: { lat_deg: 33.6321, lon_deg: -117.2510, alt_m: 3985 },
        baroAlt_ft: 13074,
        groundtrack_degT: 180,
        groundspeed_kmph: 22
      }
    ];

    it('should interpolate position at mid-point', () => {
      const result = interpolatePosition(timeSeries, 0.5);
      expect(result.location.lat_deg).toBeCloseTo(33.63205, 6);
      expect(result.location.alt_m).toBeCloseTo(3992.5, 1);
      expect(result.baroAlt_ft).toBeCloseTo(13098.5, 1);
    });

    it('should flag large gaps as interpolated', () => {
      const gappyData = [
        { ...timeSeries[0], timeOffset: 0 },
        { ...timeSeries[1], timeOffset: 2 } // 2 second gap
      ];
      const result = interpolatePosition(gappyData, 1);
      expect(result.isInterpolated).toBe(true);
    });

    it('should calculate vertical speed', () => {
      const result = interpolatePosition(timeSeries, 0.5);
      // 49 ft in 1 second = 14.9352 m/s
      expect(result.verticalSpeed_mps).toBeCloseTo(14.9352, 1);
    });
  });
});