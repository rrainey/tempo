import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const querySchema = z.object({
  limit: z.string().transform(Number).pipe(z.number().min(1).max(50)).optional().default(10),
  offset: z.string().transform(Number).pipe(z.number().min(0)).optional().default(0),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { limit, offset } = querySchema.parse(req.query);

    // Get total count
    const totalCount = await prisma.jumpLog.count({
      where: {
        userId: req.user!.id,
      },
    });

    // Get paginated jumps
    const jumps = await prisma.jumpLog.findMany({
      where: {
        userId: req.user!.id,
      },
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
      device: jump.device,
      createdAt: jump.createdAt,
      flags: jump.flags as any,
      visibleToConnections: jump.visibleToConnections,
      // Analysis data will be added later
      exitTime: null,
      deploymentAltitude: null,
      freefallTime: null,
      averageFallRate: null,
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

    console.error('Get my jumps error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
