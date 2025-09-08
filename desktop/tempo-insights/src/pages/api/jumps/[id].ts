import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { id } = req.query;

  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid jump ID' });
  }

  try {
    // Get the jump with all relations
    const jump = await prisma.jumpLog.findUnique({
      where: { id },
      include: {
        device: {
          select: {
            name: true,
            bluetoothId: true,
          },
        },
        user: {
          select: {
            id: true,
            name: true,
            slug: true,
          },
        },
      },
    });

    if (!jump) {
      return res.status(404).json({ error: 'Jump not found' });
    }

    // Check if user is authorized to view this jump
    const isOwner = jump.userId === req.user!.id;

    if (!isOwner) {
      // Check if jumps are visible to connections
      if (!jump.visibleToConnections) {
        return res.status(403).json({ error: 'This jump is private' });
      }

      // TODO: In Phase 15, we'll add connection checks here
      // For now, allow viewing if visibleToConnections is true

      // Check if they're in the same group
      const sharedGroup = await prisma.group.findFirst({
        where: {
          members: {
            some: { userId: jump.userId },
          },
          AND: {
            members: {
              some: { userId: req.user!.id },
            },
          },
        },
      });

      if (!sharedGroup) {
        return res.status(403).json({ error: 'You must be connected to view this jump' });
      }
    }

    // Parse analysis data from offsets if available
    const analysisData = jump.offsets as any || {};

    // Format response
    const jumpDetails = {
      id: jump.id,
      hash: jump.hash,
      device: jump.device,
      user: {
        id: jump.user.id,
        name: jump.user.name,
        slug: jump.user.slug,
      },
      createdAt: jump.createdAt,
      updatedAt: jump.updatedAt,
      flags: jump.flags as any,
      visibleToConnections: jump.visibleToConnections,
      isOwner,
      // Analysis fields (will be populated by analysis worker)
      exitTimestamp: analysisData.exitTimestamp || null,
      exitAltitude: analysisData.exitAltitude || null,
      deploymentAltitude: analysisData.deploymentAltitude || null,
      landingTimestamp: analysisData.landingTimestamp || null,
      freefallTime: analysisData.freefallTime || null,
      averageFallRate: analysisData.averageFallRate || null,
      maxSpeed: analysisData.maxSpeed || null,
      notes: jump.notes || null,
    };

    return res.status(200).json({
      jump: jumpDetails,
    });
  } catch (error) {
    console.error('Get jump detail error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
