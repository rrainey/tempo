// pages/api/lending/reclaim.ts
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { deviceId } = req.body;
  const userId = req.user!.id;

  if (!deviceId || typeof deviceId !== 'string') {
    return res.status(400).json({ error: 'Device ID is required' });
  }

  try {
    // Get the device with current lending info
    const device = await prisma.device.findUnique({
      where: { id: deviceId },
      include: {
        owner: {
          select: { id: true, name: true, email: true },
        },
        lentTo: {
          select: { id: true, name: true, email: true, isProxy: true },
        },
      },
    });

    if (!device) {
      return res.status(404).json({ error: 'Device not found' });
    }

    // Check ownership - only owner or admin can reclaim
    if (device.ownerId !== userId) {
      // Check if user is admin
      const user = await prisma.user.findUnique({
        where: { id: userId },
      });
      
      if (user?.role !== 'ADMIN' && user?.role !== 'SUPER_ADMIN') {
        return res.status(403).json({ error: 'Only the device owner can reclaim it' });
      }
    }

    if (!device.lentToId) {
      return res.status(400).json({ error: 'Device is not currently lent' });
    }

    const borrowerName = device.lentTo?.name;

    // Clear lending information
    const updatedDevice = await prisma.device.update({
      where: { id: deviceId },
      data: {
        lentToId: null,
        lendingDuration: null,
        lendingStartedAt: null,
      },
      include: {
        owner: {
          select: { id: true, name: true, email: true, slug: true },
        },
      },
    });

    // TODO: In Phase 8/13, this will also reassign the device back to owner on the physical device
    console.log(`[RECLAIM] Device ${device.name} reclaimed from ${borrowerName} back to ${device.owner.name}`);

    return res.status(200).json({
      success: true,
      message: `Device reclaimed successfully from ${borrowerName}`,
      device: updatedDevice,
    });
  } catch (error) {
    console.error('Reclaim device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});