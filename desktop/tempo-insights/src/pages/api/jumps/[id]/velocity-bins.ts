// pages/api/jumps/[id]/velocity-bins.ts

import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { createClient } from '@supabase/supabase-js';
import { LogParser } from '../../../../lib/analysis/log-parser';

const prisma = new PrismaClient();
const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

interface VelocityBin {
  fallRate_mph: number;
  elapsed_sec: number;
  calibrated_elapsed_sec: number;
}

// Fall Rate Calibration Factor table from coordinate-frames.md
// Altitude in feet MSL, calibration factor
const CALIBRATION_TABLE: Array<[number, number]> = [
  [20000, 0.8107],
  [18000, 0.8385],
  [16000, 0.8667],
  [14000, 0.8955],
  [12000, 0.9247],
  [10000, 0.9545],
  [9000, 0.9695],
  [8000, 0.9847],
  [7000, 1.0000],  // Reference altitude
  [6000, 1.0154],
  [5000, 1.0310],
  [4000, 1.0467],
  [3000, 1.0625],
  [2000, 1.0784],
  [1000, 1.0945],
  [0, 1.1107]
];

/**
 * Get fall rate calibration factor for a given altitude using linear interpolation
 * @param altitude_ft Altitude in feet MSL
 * @returns Calibration factor to convert raw fall rate to normalized fall rate
 */
function getCalibrationFactor(altitude_ft: number): number {
  // Handle edge cases
  if (altitude_ft >= CALIBRATION_TABLE[0][0]) {
    return CALIBRATION_TABLE[0][1];
  }
  if (altitude_ft <= CALIBRATION_TABLE[CALIBRATION_TABLE.length - 1][0]) {
    return CALIBRATION_TABLE[CALIBRATION_TABLE.length - 1][1];
  }

  // Find bracketing altitudes for linear interpolation
  for (let i = 0; i < CALIBRATION_TABLE.length - 1; i++) {
    const [alt_upper, factor_upper] = CALIBRATION_TABLE[i];
    const [alt_lower, factor_lower] = CALIBRATION_TABLE[i + 1];

    if (altitude_ft <= alt_upper && altitude_ft >= alt_lower) {
      // Linear interpolation
      const t = (altitude_ft - alt_lower) / (alt_upper - alt_lower);
      return factor_lower + t * (factor_upper - factor_lower);
    }
  }

  // Fallback (shouldn't reach here)
  return 1.0;
}

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    res.setHeader('Allow', ['GET']);
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { id } = req.query;
  
  if (!id || typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid jump ID' });
  }

  try {
    // Fetch jump log with analysis data
    const jumpLog = await prisma.jumpLog.findUnique({
      where: { id },
      select: {
        id: true,
        userId: true,
        storagePath: true,
        exitOffsetSec: true,
        deploymentOffsetSec: true,
        visibleToConnections: true,
        user: {
          select: {
            id: true,
          },
        },
      },
    });

    if (!jumpLog) {
      return res.status(404).json({ error: 'Jump not found' });
    }

    // Check access permissions
    const isOwner = req.user!.id === jumpLog.userId;
    if (!isOwner && !jumpLog.visibleToConnections) {
      return res.status(403).json({ error: 'Access denied' });
    }

    // Check if analysis has been completed
    if (!jumpLog.exitOffsetSec || !jumpLog.deploymentOffsetSec) {
      return res.status(400).json({ 
        error: 'Jump analysis incomplete',
        message: 'Exit and deployment events must be detected before velocity bin analysis' 
      });
    }

    // Fetch raw log from Supabase storage
    if (!jumpLog.storagePath) {
      return res.status(404).json({ error: 'Jump log file not found' });
    }

    const { data, error } = await supabase.storage
      .from('jump-logs')
      .download(jumpLog.storagePath);

    if (error || !data) {
      console.error('Error downloading jump log:', error);
      return res.status(500).json({ error: 'Failed to retrieve jump log data' });
    }

    // Convert blob to buffer
    const buffer = Buffer.from(await data.arrayBuffer());

    // Parse the log
    const parsedData = LogParser.parseLog(buffer);
    
    if (!parsedData.hasValidData || parsedData.logEntries.length === 0) {
      return res.status(400).json({ error: 'Invalid log data' });
    }

    // Initialize velocity bins (90-200 mph)
    const velocityBins: Map<number, VelocityBin> = new Map();
    for (let mph = 90; mph <= 200; mph++) {
      velocityBins.set(mph, { 
        fallRate_mph: mph, 
        elapsed_sec: 0.0,
        calibrated_elapsed_sec: 0.0
      });
    }

    // Define analysis window
    const analysisStartTime = jumpLog.exitOffsetSec + 12.0; // 12 seconds after exit
    const analysisEndTime = jumpLog.deploymentOffsetSec - 2.0; // 2 seconds before deployment

    // Process log entries
    let lastTimestamp: number | null = null;
    
    for (const entry of parsedData.logEntries) {
      // Skip entries outside analysis window
      if (entry.timeOffset < analysisStartTime || entry.timeOffset > analysisEndTime) {
        continue;
      }

      // Skip entries without rate of descent or altitude data
      if (entry.rateOfDescent_fpm === null || entry.rateOfDescent_fpm === undefined) {
        continue;
      }

      // Calculate deltaT_sec
      let deltaT_sec = 0;
      if (lastTimestamp !== null) {
        deltaT_sec = entry.timeOffset - lastTimestamp;
      }
      lastTimestamp = entry.timeOffset;

      // Skip first entry (no deltaT yet)
      if (deltaT_sec === 0) {
        continue;
      }

      // Convert fpm to mph (raw fall rate)
      const fallRate_fpm = entry.rateOfDescent_fpm;
      const rawFallRate_mph = Math.round(fallRate_fpm * 60 / 5280);

      // Get calibration factor for current altitude
      let alt_ft = entry.baroAlt_ft || (entry.location?.alt_m || 0) * 3.28084;
      const calibrationFactor = getCalibrationFactor(alt_ft);
      
      // Calculate calibrated fall rate
      // TODO: weshould reject entries lacking good altitude data
      const calibratedFallRate_mph = Math.round(rawFallRate_mph * calibrationFactor);

      // Accumulate time in raw fall rate bin
      if (rawFallRate_mph >= 90 && rawFallRate_mph <= 200) {
        const rawBin = velocityBins.get(rawFallRate_mph);
        if (rawBin) {
          rawBin.elapsed_sec += deltaT_sec;
        }
      }

      // Accumulate time in calibrated fall rate bin
      if (calibratedFallRate_mph >= 90 && calibratedFallRate_mph <= 200) {
        const calibratedBin = velocityBins.get(calibratedFallRate_mph);
        if (calibratedBin) {
          calibratedBin.calibrated_elapsed_sec += deltaT_sec;
        }
      }
    }

    // Filter out bins with 0 elapsed time in BOTH categories and convert to array
    const results = Array.from(velocityBins.values())
      .filter(bin => bin.elapsed_sec > 0 || bin.calibrated_elapsed_sec > 0)
      .sort((a, b) => a.fallRate_mph - b.fallRate_mph);

    // Calculate summary statistics for both raw and calibrated
    const totalRawTime = results.reduce((sum, bin) => sum + bin.elapsed_sec, 0);
    const totalCalibratedTime = results.reduce((sum, bin) => sum + bin.calibrated_elapsed_sec, 0);
    
    const avgRawFallRate = totalRawTime > 0 
      ? results.reduce((sum, bin) => sum + (bin.fallRate_mph * bin.elapsed_sec), 0) / totalRawTime
      : 0;
    
    const avgCalibratedFallRate = totalCalibratedTime > 0
      ? results.reduce((sum, bin) => sum + (bin.fallRate_mph * bin.calibrated_elapsed_sec), 0) / totalCalibratedTime
      : 0;

    // Find min/max for bins with non-zero time
    const binsWithRawTime = results.filter(bin => bin.elapsed_sec > 0);
    const binsWithCalibratedTime = results.filter(bin => bin.calibrated_elapsed_sec > 0);

    return res.status(200).json({
      velocityBins: results,
      summary: {
        raw: {
          totalAnalysisTime: totalRawTime,
          averageFallRate: Math.round(avgRawFallRate),
          minFallRate: binsWithRawTime.length > 0 ? binsWithRawTime[0].fallRate_mph : null,
          maxFallRate: binsWithRawTime.length > 0 ? binsWithRawTime[binsWithRawTime.length - 1].fallRate_mph : null,
        },
        calibrated: {
          totalAnalysisTime: totalCalibratedTime,
          averageFallRate: Math.round(avgCalibratedFallRate),
          minFallRate: binsWithCalibratedTime.length > 0 ? binsWithCalibratedTime[0].fallRate_mph : null,
          maxFallRate: binsWithCalibratedTime.length > 0 ? binsWithCalibratedTime[binsWithCalibratedTime.length - 1].fallRate_mph : null,
        },
        analysisWindow: {
          startOffset: analysisStartTime,
          endOffset: analysisEndTime,
          duration: analysisEndTime - analysisStartTime
        }
      }
    });

  } catch (error) {
    console.error('Error in velocity bins analysis:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});