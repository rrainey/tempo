// pages/api/lending/lend.ts
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';

const prisma = new PrismaClient();

export enum LendingDuration {
  ONE_JUMP = 'ONE_JUMP',
  UNTIL_RECLAIM = 'UNTIL_RECLAIM'
}

interface LendDeviceRequest {
  deviceId: string;
  targetUserId?: string;
  newUserName?: string;
  duration: LendingDuration;
}

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { deviceId, targetUserId, newUserName, duration }: LendDeviceRequest = req.body;
  const ownerId = req.user!.id;

  // Validate input
  if (!deviceId || !duration) {
    return res.status(400).json({ error: 'Device ID and duration are required' });
  }

  if (!targetUserId && !newUserName) {
    return res.status(400).json({ error: 'Either target user ID or new user name is required' });
  }

  if (targetUserId && newUserName) {
    return res.status(400).json({ error: 'Cannot specify both target user ID and new user name' });
  }

  try {
    // Check if the device exists and belongs to the user
    const device = await prisma.device.findUnique({
      where: { id: deviceId },
      include: {
        owner: true,
        lentTo: true,
      },
    });

    if (!device) {
      return res.status(404).json({ error: 'Device not found' });
    }

    if (device.ownerId !== ownerId) {
      return res.status(403).json({ error: 'You can only lend your own devices' });
    }

    if (device.lentToId) {
      return res.status(400).json({ 
        error: `Device is already lent to ${device.lentTo?.name}. Reclaim it first before lending again.` 
      });
    }

    let lentToUserId: string;

    // Handle lending to new user (create proxy)
    if (newUserName) {
      // Create proxy user via internal API call
      const proxyResponse = await fetch(`${process.env.NEXTAUTH_URL || 'http://localhost:3000'}/api/lending/proxy`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Cookie': req.headers.cookie || '',
        },
        body: JSON.stringify({ name: newUserName }),
      });

      if (!proxyResponse.ok) {
        const error = await proxyResponse.json();
        return res.status(400).json({ error: error.error || 'Failed to create proxy user' });
      }

      const { proxyUser } = await proxyResponse.json();
      lentToUserId = proxyUser.id;
    } else {
      // Verify target user exists
      const targetUser = await prisma.user.findUnique({
        where: { id: targetUserId },
      });

      if (!targetUser) {
        return res.status(404).json({ error: 'Target user not found' });
      }

      lentToUserId = targetUserId!;
    }

    // Update device with lending information
    const updatedDevice = await prisma.device.update({
      where: { id: deviceId },
      data: {
        lentToId: lentToUserId,
        lendingDuration: duration,
        lendingStartedAt: new Date(),
      },
      include: {
        owner: {
          select: { id: true, name: true, email: true, slug: true },
        },
        lentTo: {
          select: { id: true, name: true, email: true, slug: true, isProxy: true },
        },
      },
    });

    // TODO: In Phase 8/13, this will also update the device's on-device user assignment
    console.log(`[LENDING] Device ${device.name} lent from ${device.owner.name} to ${updatedDevice.lentTo?.name}`);

    return res.status(200).json({
      success: true,
      message: `Device lent successfully`,
      device: updatedDevice,
    });
  } catch (error) {
    console.error('Lend device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});