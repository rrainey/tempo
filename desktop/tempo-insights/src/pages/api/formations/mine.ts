import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const querySchema = z.object({
  limit: z.string().transform(Number).pipe(z.number().min(1).max(20)).optional().default(5),
  offset: z.string().transform(Number).pipe(z.number().min(0)).optional().default(0),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { limit, offset } = querySchema.parse(req.query);

    // Get total count of formations user is in
    const totalCount = await prisma.formationParticipant.count({
      where: {
        userId: req.user!.id,
      },
    });

    // Get formations the user participates in
    const formations = await prisma.formationSkydive.findMany({
      where: {
        participants: {
          some: {
            userId: req.user!.id,
          },
        },
      },
      include: {
        participants: {
          include: {
            user: {
              select: {
                id: true,
                name: true,
                email: true,
                slug: true,
              },
            },
            jumpLog: {
              select: {
                id: true,
                exitTimestampUTC: true,
                exitAltitudeFt: true,
                freefallTimeSec: true,
                visibleToConnections: true,
              },
            },
          },
          orderBy: {
            position: 'asc',
          },
        },
      },
      orderBy: {
        jumpTime: 'desc',
      },
      take: limit,
      skip: offset,
    });

    // Format formations for response
    const formattedFormations = formations.map(formation => ({
      id: formation.id,
      name: formation.name,
      jumpTime: formation.jumpTime,
      altitude: formation.altitude,
      notes: formation.notes,
      isPublic: formation.isPublic,
      createdAt: formation.createdAt,
      
      // Participants info
      participantCount: formation.participants.length,
      participants: formation.participants.map(p => ({
        position: p.position,
        userId: p.user.id,
        userName: p.user.name || p.user.email,
        userSlug: p.user.slug,
        jumpLogId: p.jumpLog.id,
        
        // Include jump data only if visible or own jump
        exitTime: (p.jumpLog.visibleToConnections || p.userId === req.user!.id) 
          ? p.jumpLog.exitTimestampUTC : null,
        exitAltitude: (p.jumpLog.visibleToConnections || p.userId === req.user!.id)
          ? p.jumpLog.exitAltitudeFt : null,
        freefallTime: (p.jumpLog.visibleToConnections || p.userId === req.user!.id)
          ? p.jumpLog.freefallTimeSec : null,
        jumpVisible: p.jumpLog.visibleToConnections,
      })),
      
      // User's position in this formation
      myPosition: formation.participants.find(p => p.userId === req.user!.id)?.position || null,
    }));

    return res.status(200).json({
      formations: formattedFormations,
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

    console.error('Get formations error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});