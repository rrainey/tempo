// Save as: src/pages/api/devices/[id]/commands/led-on.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../../lib/auth/middleware';
import { PrismaClient, CommandType } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const bodySchema = z.object({
  color: z.enum(['red', 'green', 'blue', 'orange', 'yellow', 'purple', 'white']).optional().default('orange')
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { id: deviceId } = req.query;
    
    if (!deviceId || typeof deviceId !== 'string') {
      return res.status(400).json({ error: 'Invalid device ID' });
    }

    // Parse and validate request body
    const body = bodySchema.parse(req.body);

    // Get the device and check permissions
    const device = await prisma.device.findUnique({
      where: { id: deviceId },
      include: {
        owner: true,
        lentTo: true
      }
    });

    if (!device) {
      return res.status(404).json({ error: 'Device not found' });
    }

    // Check if user has permission
    const isAdmin = req.user!.role === 'ADMIN' || req.user!.role === 'SUPER_ADMIN';
    const isOwner = device.ownerId === req.user!.id;
    const isBorrower = device.lentToId === req.user!.id;

    if (!isAdmin && !isOwner && !isBorrower) {
      return res.status(403).json({ error: 'You do not have permission to control this device' });
    }

    // Check if device is online
    const cutoffTime = new Date(Date.now() - 5 * 60 * 1000); // 5 minutes
    const isOnline = device.lastSeen && device.lastSeen > cutoffTime;

    if (!isOnline) {
      return res.status(400).json({ error: 'Device is offline' });
    }

    // Create the command in the queue
    const command = await prisma.deviceCommandQueue.create({
      data: {
        commandType: CommandType.BLINK_ON,
        targetDeviceId: deviceId,
        sendingUserId: req.user!.id,
        commandData: { color: body.color }
      }
    });

    return res.status(200).json({
      message: `LED command queued for ${device.name}`,
      commandId: command.id,
      color: body.color
    });

  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({ error: 'Invalid request data', details: error });
    }

    console.error('LED on error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});