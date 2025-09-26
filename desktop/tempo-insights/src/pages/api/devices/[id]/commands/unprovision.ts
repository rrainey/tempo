// /pages/api/devices/[id]/commands/unprovision.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../../lib/auth/middleware';
import { PrismaClient, CommandType } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { id: deviceId } = req.query;
    
    if (!deviceId || typeof deviceId !== 'string') {
      return res.status(400).json({ error: 'Invalid device ID' });
    }

    // Only admins can unprovision devices
    if (req.user!.role !== 'ADMIN' && req.user!.role !== 'SUPER_ADMIN') {
      return res.status(403).json({ error: 'Only administrators can unprovision devices' });
    }

    // Get the device
    const device = await prisma.device.findUnique({
      where: { id: deviceId },
      include: {
        owner: {
          select: {
            id: true,
            name: true
          }
        },
        lentTo: {
          select: {
            id: true,
            name: true
          }
        }
      }
    });

    if (!device) {
      return res.status(404).json({ error: 'Device not found' });
    }

    // Check if device is online
    const cutoffTime = new Date(Date.now() - 5 * 60 * 1000); // 5 minutes
    const isOnline = device.lastSeen && device.lastSeen > cutoffTime;

    if (!isOnline) {
      return res.status(400).json({ error: 'Device is offline' });
    }

    // Check if device is lent out
    if (device.lentToId) {
      return res.status(400).json({ 
        error: 'Cannot unprovision a lent device. Please reclaim it first.' 
      });
    }

    // Create the command in the queue
    const command = await prisma.deviceCommandQueue.create({
      data: {
        commandType: CommandType.UNPROVISION,
        targetDeviceId: deviceId,
        sendingUserId: req.user!.id,
        commandData: {
          previousOwner: device.owner ? {
            id: device.owner.id,
            name: device.owner.name
          } : null
        }
      }
    });

    // Update device state in database
    await prisma.device.update({
      where: { id: deviceId },
      data: {
        state: 'PROVISIONING',
        // Don't clear the owner yet - wait for command to complete
      }
    });

    return res.status(200).json({
      message: `Unprovision command queued for ${device.name}`,
      commandId: command.id,
      previousOwner: device.owner?.name || 'None'
    });

  } catch (error) {
    console.error('Unprovision device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});