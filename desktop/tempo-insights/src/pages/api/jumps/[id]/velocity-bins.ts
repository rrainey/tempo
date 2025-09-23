// pages/api/jumps/[id]/velocity-bins.ts

import { NextApiRequest, NextApiResponse } from 'next';
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
      velocityBins.set(mph, { fallRate_mph: mph, elapsed_sec: 0.0 });
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

      // Skip entries without rate of descent data
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

      // Convert fpm to mph and round to nearest integer
      const fallRate_fpm = entry.rateOfDescent_fpm;
      const fallRate_mph = Math.round(fallRate_fpm * 60 / 5280); // fpm * 60 min/hr / 5280 ft/mile

      // Only consider rates in our bin range
      if (fallRate_mph >= 90 && fallRate_mph <= 200) {
        const bin = velocityBins.get(fallRate_mph);
        if (bin) {
          bin.elapsed_sec += deltaT_sec;
        }
      }
    }

    // Filter out bins with 0 elapsed time and convert to array
    const results = Array.from(velocityBins.values())
      .filter(bin => bin.elapsed_sec > 0)
      .sort((a, b) => a.fallRate_mph - b.fallRate_mph);

    // Calculate some summary statistics
    const totalTime = results.reduce((sum, bin) => sum + bin.elapsed_sec, 0);
    const avgFallRate = totalTime > 0 
      ? results.reduce((sum, bin) => sum + (bin.fallRate_mph * bin.elapsed_sec), 0) / totalTime
      : 0;

    return res.status(200).json({
      velocityBins: results,
      summary: {
        totalAnalysisTime: totalTime,
        averageFallRate: Math.round(avgFallRate),
        minFallRate: results.length > 0 ? results[0].fallRate_mph : null,
        maxFallRate: results.length > 0 ? results[results.length - 1].fallRate_mph : null,
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