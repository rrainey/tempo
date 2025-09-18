// pages/api/devices/mine.ts
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const userId = req.user!.id;

  try {
    // Get devices owned by the user
    const devices = await prisma.device.findMany({
      where: {
        ownerId: userId,
      },
      include: {
        lentTo: {
          select: {
            id: true,
            name: true,
            email: true,
            slug: true,
          },
        },
        _count: {
          select: {
            jumpLogs: true,
          },
        },
      },
      orderBy: {
        name: 'asc',
      },
    });

    // Calculate online status
    const devicesWithStatus = devices.map(device => ({
      ...device,
      isOnline: device.lastSeen
        ? new Date().getTime() - new Date(device.lastSeen).getTime() < 5 * 60 * 1000
        : false,
      jumpCount: device._count.jumpLogs,
    }));

    return res.status(200).json({
      devices: devicesWithStatus,
    });
  } catch (error) {
    console.error('Get my devices error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});