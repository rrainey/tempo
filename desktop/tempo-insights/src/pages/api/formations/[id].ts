// pages/api/formations/[id].ts
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

// Mock time series data generator for testing
// TODO: Replace with actual log parsing in Phase 11 completion
function generateMockTimeSeries(userId: string, jumpLogId: string, index: number) {
  const timeSeries = [];
  const duration = 60; // 60 seconds of freefall
  const sampleRate = 4; // 4Hz GPS
  
  // Starting positions offset by index for formation visualization
  const startLat = 33.6320 + (index * 0.0001);
  const startLon = -117.2510 + (index * 0.0001);
  const startAlt = 4267; // 14000 ft in meters
  
  for (let t = 0; t < duration; t += 1/sampleRate) {
    timeSeries.push({
      timeOffset: t,
      location: {
        lat_deg: startLat + (Math.sin(t * 0.1) * 0.0002),
        lon_deg: startLon + (Math.cos(t * 0.1) * 0.0002),
        alt_m: startAlt - (t * 15), // ~50ft/s fall rate
      },
      baroAlt_ft: Math.round((startAlt - (t * 15)) * 3.28084),
      groundspeed_kmph: 20 + Math.random() * 10,
      groundtrack_degT: 180 + Math.sin(t * 0.05) * 30,
      verticalSpeed_mps: -15 + Math.random() * 2,
      normalizedFallRate_mph: 120 + Math.random() * 10,
    });
  }
  
  return timeSeries;
}

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { id } = req.query;

  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid formation ID' });
  }

  try {
    // Get the formation with all participants
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
                slug: true,
              },
            },
            jumpLog: {
              select: {
                id: true,
                hash: true,
                rawLog: false, // Don't include raw bytes here
                offsets: true,
                flags: true,
                visibleToConnections: true,
                exitTimestampUTC: true,
                exitLatitude: true,
                exitLongitude: true,
                exitAltitudeFt: true,
                freefallTimeSec: true,
                avgFallRateMph: true,
              },
            },
          },
          orderBy: {
            position: 'asc',
          },
        },
      },
    });

    if (!formation) {
      return res.status(404).json({ error: 'Formation not found' });
    }

    // Check if user is authorized to view this formation
    const isParticipant = formation.participants.some(p => p.userId === req.user!.id);
    
    if (!isParticipant && !formation.isPublic) {
      // Check if user is in a group with any participant
      const participantIds = formation.participants.map(p => p.userId);
      const sharedGroup = await prisma.group.findFirst({
        where: {
          members: {
            some: { userId: req.user!.id },
          },
          AND: {
            members: {
              some: { 
                userId: { in: participantIds },
              },
            },
          },
        },
      });

      if (!sharedGroup && req.user!.role !== 'ADMIN' && req.user!.role !== 'SUPER_ADMIN') {
        return res.status(403).json({ error: 'You are not authorized to view this formation' });
      }
    }

    // Determine base jumper - for now, use participant at position 1
    // TODO: Store baseJumperId in formation metadata
    const baseParticipant = formation.participants.find(p => p.position === 1);
    const baseJumperId = baseParticipant?.userId || formation.participants[0]?.userId;

    // Calculate jump run track from base jumper's data
    // TODO: Extract from actual log data
    const jumpRunTrack_degTrue = 180; // Mock value

    // Build participant data with time series
    const participantsData = formation.participants.map((participant, index) => {
      // Check visibility rules
      const isOwnJump = participant.userId === req.user!.id;
      const isVisible = isOwnJump || participant.jumpLog.visibleToConnections;
      
      // Generate mock time series for now
      // TODO: Parse actual log data from rawLog bytes
      const timeSeries = isVisible ? 
        generateMockTimeSeries(participant.userId, participant.jumpLogId, index) : 
        [];

      // Assign colors for visualization
      const colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#DDA0DD', '#98D8C8'];
      const color = colors[index % colors.length];

      return {
        userId: participant.userId,
        jumpLogId: participant.jumpLogId,
        name: participant.user.name || participant.user.slug,
        color,
        isBase: participant.userId === baseJumperId,
        isVisible,
        timeSeries,
      };
    });

    // Filter out participants with no visible data unless they're the requesting user
    const visibleParticipants = participantsData.filter(p => 
      p.isVisible || p.userId === req.user!.id
    );

    // Format response
    const formationData = {
      id: formation.id,
      startTime: formation.jumpTime,
      baseJumperId,
      jumpRunTrack_degTrue,
      participants: visibleParticipants,
      dzElevation_m: formation.dropzone?.elevation || null,
    };

    return res.status(200).json(formationData);
  } catch (error) {
    console.error('Get formation data error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});