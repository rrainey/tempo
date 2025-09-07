import { requireAdmin, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default requireAdmin(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Get all devices with owner and lentTo information
    const devices = await prisma.device.findMany({
      include: {
        owner: {
          select: {
            id: true,
            name: true,
            email: true,
            slug: true,
          },
        },
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
        lastSeen: 'desc',
      },
    });

    // Calculate online status (device is online if lastSeen within 5 minutes)
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
    console.error('List devices error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
