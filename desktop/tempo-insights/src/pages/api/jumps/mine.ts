// src/pages/api/jumps/mine.ts
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const querySchema = z.object({
  limit: z.string().transform(Number).pipe(z.number().min(1).max(50)).optional().default(10),
  offset: z.string().transform(Number).pipe(z.number().min(0)).optional().default(0),
  targetUserId: z.string().optional(),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { limit, offset, targetUserId } = querySchema.parse(req.query);
    const currentUserId = req.user!.id;
    const isAdmin = req.user!.role === 'ADMIN' || req.user!.role === 'SUPER_ADMIN';

    // Determine which user's jumps to fetch
    const userId = targetUserId || currentUserId;
    const isViewingOwnJumps = userId === currentUserId;

    // Build where clause based on visibility rules
    let whereClause: any = { userId };

    // If viewing another user's jumps, apply visibility rules
    if (!isViewingOwnJumps) {
      // Admins can see all jumps
      if (!isAdmin) {
        // Regular users must check visibility
        whereClause.visibleToConnections = true;

        // Check if users are connected or share a group
        const [isConnected, shareGroup] = await Promise.all([
          prisma.connection.findFirst({
            where: {
              OR: [
                { userId1: currentUserId, userId2: userId },
                { userId1: userId, userId2: currentUserId }
              ]
            }
          }),
          prisma.groupMember.findFirst({
            where: {
              userId: currentUserId,
              group: {
                members: {
                  some: { userId }
                }
              }
            }
          })
        ]);

        if (!isConnected && !shareGroup) {
          return res.status(403).json({ 
            error: 'You must be connected with this user or share a group to view their jumps' 
          });
        }
      }
    }

    // Get total count
    const totalCount = await prisma.jumpLog.count({ where: whereClause });

    // Get paginated jumps
    const jumps = await prisma.jumpLog.findMany({
      where: whereClause,
      include: {
        device: {
          select: {
            name: true,
            bluetoothId: true,
          },
        },
      },
      orderBy: {
        createdAt: 'desc',
      },
      take: limit,
      skip: offset,
    });

    // Format jumps for response
    const formattedJumps = jumps.map(jump => ({
      id: jump.id,
      hash: jump.hash,
      jumpNumber: jump.jumpNumber,
      deviceName: jump.device.name,
      createdAt: jump.createdAt,
      
      // Analysis status
      analyzed: jump.initialAnalysisTimestamp !== null,
      hasIssues: jump.initialAnalysisMessage !== null,
      message: jump.initialAnalysisMessage,
      
      // Analysis data
      exitTimestamp: jump.exitTimestampUTC,
      exitAltitude: jump.exitAltitudeFt,
      deployAltitude: jump.deployAltitudeFt,
      freefallTime: jump.freefallTimeSec,
      avgFallRate: jump.avgFallRateMph,
      
      // GPS
      hasGPS: jump.exitLatitude !== null && jump.exitLongitude !== null,
      exitLocation: jump.exitLatitude && jump.exitLongitude ? {
        lat: jump.exitLatitude,
        lng: jump.exitLongitude
      } : null,
      
      // Visibility and notes
      visible: jump.visibleToConnections,
      notes: jump.notes,
      
      // Keep original fields for compatibility
      flags: jump.flags as any,
      visibleToConnections: jump.visibleToConnections,
    }));

    return res.status(200).json({
      jumps: formattedJumps,
      pagination: {
        total: totalCount,
        limit,
        offset,
        hasMore: offset + limit < totalCount,
      },
    });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({
        error: 'Invalid query parameters',
        details: error.issues,
      });
    }

    console.error('Get jumps error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});