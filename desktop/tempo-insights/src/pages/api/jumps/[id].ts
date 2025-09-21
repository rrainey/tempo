// pages/api/jumps/[id].ts

import { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { LogParser } from '../../../lib/analysis/log-parser';

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
    return res.status(400).json({ error: 'Invalid jump ID' });
  }

  try {
    // First, get the jump
    const jump = await prisma.jumpLog.findUnique({
      where: { id },
      include: {
        device: {
          select: {
            name: true
          },
        },        user: {
          select: {
            id: true,
            name: true,
            email: true,
            slug: true
          }
        }
      }
    });

    if (!jump) {
      return res.status(404).json({ error: 'Jump not found' });
    }

    const isOwner = jump.userId === req.user.id;
    
    // Check visibility permissions
    if (!isOwner && !jump.visibleToConnections) {
      return res.status(403).json({ error: 'This jump is private' });
    }
    
    // If not owner but jump is visible to connections, check if they share a formation
    if (!isOwner && jump.visibleToConnections) {
      // Check if they've been in any formation together
      const sharedFormation = await prisma.formationParticipant.findFirst({
        where: {
          AND: [
            {
              formation: {
                participants: {
                  some: {
                    userId: req.user.id
                  }
                }
              }
            },
            {
              formation: {
                participants: {
                  some: {
                    userId: jump.userId
                  }
                }
              }
            }
          ]
        }
      });

      if (!sharedFormation) {
        return res.status(403).json({ error: 'You must have jumped together to view this jump' });
      }
    }

    // Parse the log to get time series data if available
    let timeSeries = null;
    if (jump.rawLog) {
      try {
        // Convert Uint8Array to Buffer
        const buffer = Buffer.from(jump.rawLog);
        const parsedData = LogParser.parseLog(buffer);
        
        // Extract time series for the chart
        timeSeries = {
          altitude: parsedData.altitude,
          vspeed: parsedData.vspeed,
          gps: parsedData.gps,
          duration: parsedData.duration,
          sampleRate: parsedData.sampleRate,
          hasGPS: parsedData.hasGPS,
          
          // Include event times from analysis
          exitOffsetSec: jump.exitOffsetSec || undefined,
          deploymentOffsetSec: jump.deploymentOffsetSec || undefined,
          landingOffsetSec: jump.landingOffsetSec || undefined,
        };
      } catch (error) {
        console.error('Failed to parse log for time series:', error);
      }
    }

    // Construct response with analysis data
    const jumpData = {
      id: jump.id,
      hash: jump.hash,
      device: {
        name: jump.device.name,
        bluetoothId: jump.device.bluetoothId
      },
      user: jump.user,
      createdAt: jump.createdAt.toISOString(),
      updatedAt: jump.updatedAt.toISOString(),
      flags: jump.flags,
      visibleToConnections: jump.visibleToConnections,
      isOwner,
      notes: jump.notes,
      
      // Analysis results
      exitTimestamp: jump.exitTimestampUTC?.toISOString() || null,
      exitAltitude: jump.exitAltitudeFt || null,
      deploymentAltitude: jump.deployAltitudeFt || null,
      landingTimestamp: jump.landingOffsetSec && jump.exitTimestampUTC 
        ? new Date(jump.exitTimestampUTC.getTime() + (jump.landingOffsetSec * 1000)).toISOString()
        : null,
      freefallTime: jump.freefallTimeSec || null,
      averageFallRate: jump.avgFallRateMph || null,
      maxSpeed: null, // Could be calculated from time series
      
      // Include time series data
      timeSeries
    };

    res.status(200).json({ jump: jumpData });
  } catch (error) {
    console.error('Error fetching jump:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);