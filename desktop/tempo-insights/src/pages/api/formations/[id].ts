// pages/api/formations/[id].ts

import { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { LogParser } from '../../../lib/analysis/log-parser';
import { createClient } from '@supabase/supabase-js';

const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

const prisma = new PrismaClient();

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  const { id } = req.query;
  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid formation ID' });
  }

  try {
    // Fetch formation with participants and their jump logs
    const formation = await prisma.formationSkydive.findUnique({
      where: { id },
      include: {
        dropzone: true,
        participants: {
          include: {
            user: {
              select: {
                id: true,
                name: true,
                email: true,
                slug: true
              }
            },
            jumpLog: {
              include: {
                device: true
              }
            }
          },
          orderBy: {
            position: 'asc'
          }
        }
      }
    });

    if (!formation) {
      return res.status(404).json({ error: 'Formation not found' });
    }

    // Check if user can view this formation
    const participantIds = formation.participants.map(p => p.userId);
    const isParticipant = participantIds.includes(req.user.id);
    
    // Only participants can view private formations
    if (!isParticipant && !formation.isPublic) {
      return res.status(403).json({ error: 'Access denied - private formation' });
    }

    // Parse each participant's log to extract time series
    const participantsWithData = await Promise.all(
      formation.participants.map(async (participant) => {
        // TODO: this could better be expressed as a type that we pass around the app
        let timeSeries: {
          timeOffset: number;
          location: { lat_deg: number; lon_deg: number; alt_m: number } | null;
          baroAlt_m: number | null;
          groundtrack_degTrue: number | undefined;
          groundspeed_mps: number | null;
          rateOfDescent_mps: number | null;
        }[] = [];
        let jumpData = null;
        let color = '#66ccff'; // Default color
        
        // Assign colors based on position
        const colors = ['#ddff55', '#855bf0', '#66ccff', '#00ff88', '#ffaa00', '#ff3355'];
        color = colors[(participant.position - 1) % colors.length];
        
        // Check if this participant's data is visible
        const isOwnData = req.user ? participant.userId === req.user.id : false;
        const isJumpVisible = participant.jumpLog.visibleToConnections;
        
        // Only include time series if user can see it
        if (isOwnData || (isParticipant && isJumpVisible)) {

          if (participant.jumpLog.storagePath) {
            try {
              // For API parsing, download and parse
              const { data: fileData, error } = await supabase.storage
                .from('jump-logs')
                .download(participant.jumpLog.storagePath);
      
              if (!error && fileData) {
                
                const buffer = Buffer.from(await fileData.arrayBuffer());
                const parsedData = LogParser.parseLog(buffer);

                const exitTime = participant.jumpLog.exitOffsetSec || 0;
                const deploymentTime = participant.jumpLog.deploymentOffsetSec || 0;
                const landingTime = participant.jumpLog.landingOffsetSec || deploymentTime + 180; // 3 min canopy default

                // Filter to relevant time window: 15 seconds before exit through landing
                const preExitBuffer = 15; // seconds before exit to include
                const startTime = Math.max(0, exitTime - preExitBuffer);
                const endTime = landingTime;
                
                // Convert KMLDataV1 entries to formation time series format
                timeSeries = parsedData.logEntries
                    .filter(entry => entry.timeOffset >= startTime && entry.timeOffset <= endTime)
                    .map(entry => ({
                  timeOffset: entry.timeOffset,
                  location: entry.location ? {
                    lat_deg: entry.location.lat_deg,
                    lon_deg: entry.location.lon_deg,
                    alt_m: entry.location.alt_m
                  } : null,
                  baroAlt_m: entry.baroAlt_ft ? entry.baroAlt_ft / 3.28084 : null,
                  groundtrack_degTrue: entry.groundtrack_degT !== null && entry.groundtrack_degT !== undefined ? entry.groundtrack_degT : undefined,
                  groundspeed_mps: entry.groundspeed_kmph ? entry.groundspeed_kmph / 3.6 : null,
                  rateOfDescent_mps: entry.rateOfDescent_fpm ? entry.rateOfDescent_fpm / 196.85 : null
                }));
              
                jumpData = {
                  exitOffsetSec: participant.jumpLog.exitOffsetSec,
                  exitAltitudeFt: participant.jumpLog.exitAltitudeFt,
                  deploymentOffsetSec: participant.jumpLog.deploymentOffsetSec,
                  deployAltitudeFt: participant.jumpLog.deployAltitudeFt,
                  landingOffsetSec: participant.jumpLog.landingOffsetSec,
                  freefallTimeSec: participant.jumpLog.freefallTimeSec,
                  avgFallRateMph: participant.jumpLog.avgFallRateMph
                };
              }
            } catch (error) {
              console.error('Failed to parse log for time series:', error);
            }
          }
        }

        return {
          userId: participant.userId,
          name: participant.user.name || participant.user.email,
          position: participant.position,
          isBase: participant.position === 1, // Position 1 is base by convention
          isVisible: isOwnData || (isParticipant && isJumpVisible),
          color,
          timeSeries,
          jumpData
        };
      })
    );

    // Determine jump run track from first two participants with GPS data
    let jumpRunTrack_degTrue = 0;
    const participantsWithGPS = participantsWithData.filter(p => 
      p.isVisible && p.timeSeries.some(ts => ts.location !== null)
    );
    
    if (participantsWithGPS.length >= 2) {
      // Find exit points for first two jumpers
      const exitPoints = participantsWithGPS.slice(0, 2).map(p => {
        const exitTime = p.jumpData?.exitOffsetSec || 0;
        const exitEntry = p.timeSeries.find(ts => Math.abs(ts.timeOffset - exitTime) < 1);
        return exitEntry?.location;
      }).filter(loc => loc !== null);
      
      if (exitPoints.length === 2) {
        // Calculate bearing between the two exit points
        const lat1 = exitPoints[0]!.lat_deg * Math.PI / 180;
        const lat2 = exitPoints[1]!.lat_deg * Math.PI / 180;
        const dLon = (exitPoints[1]!.lon_deg - exitPoints[0]!.lon_deg) * Math.PI / 180;
        
        const y = Math.sin(dLon) * Math.cos(lat2);
        const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
        
        jumpRunTrack_degTrue = (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
      }
    }

    // Use position 1 as base, or first visible participant
    const baseParticipant = participantsWithData.find(p => p.position === 1 && p.isVisible);
    const firstVisible = participantsWithData.find(p => p.isVisible);
    let baseJumperId = baseParticipant?.userId || firstVisible?.userId || '';

    // Get dropzone center if available, otherwise derive from first GPS point
    let dzCenter = null;
    if (formation.dropzone) {
      dzCenter = {
        lat_deg: formation.dropzone.latitude,
        lon_deg: formation.dropzone.longitude,
        alt_m: formation.dropzone.elevation
      };
    } else {
      // Fallback to first GPS point
      const firstGPSPoint = participantsWithGPS[0]?.timeSeries.find(ts => ts.location);
      if (firstGPSPoint?.location) {
        dzCenter = {
          lat_deg: firstGPSPoint.location.lat_deg,
          lon_deg: firstGPSPoint.location.lon_deg,
          alt_m: 436.5 // Default elevation
        };
      }
    }

    const formationData = {
      id: formation.id,
      name: formation.name,
      jumpTime: formation.jumpTime.toISOString(),
      startTime: formation.jumpTime,
      altitude: formation.altitude,
      notes: formation.notes,
      isPublic: formation.isPublic,
      baseJumperId,
      jumpRunTrack_degTrue,
      participants: participantsWithData,
      dzElevation_m: formation.dropzone?.elevation || 436.5,
      dzCenter,
      dropzone: formation.dropzone ? {
        id: formation.dropzone.id,
        name: formation.dropzone.name,
        slug: formation.dropzone.slug
      } : null,
      createdAt: formation.createdAt.toISOString(),
      updatedAt: formation.updatedAt.toISOString()
    };

    res.status(200).json(formationData);
  } catch (error) {
    console.error('Error fetching formation:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);