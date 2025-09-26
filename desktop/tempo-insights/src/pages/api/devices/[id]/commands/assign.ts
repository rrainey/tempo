// Save as: src/pages/api/devices/[id]/commands/assign.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../../lib/auth/middleware';
import { PrismaClient, CommandType } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const bodySchema = z.object({
  userId: z.string(),
  nextJumpNumber: z.number().int().positive().optional()
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

    // Only admins can assign devices
    if (req.user!.role !== 'ADMIN' && req.user!.role !== 'SUPER_ADMIN') {
      return res.status(403).json({ error: 'Only administrators can assign devices' });
    }

    // Parse and validate request body
    const body = bodySchema.parse(req.body);

    // Get the device
    const device = await prisma.device.findUnique({
      where: { id: deviceId }
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

    // Get the target user
    const targetUser = await prisma.user.findUnique({
      where: { id: body.userId },
      select: {
        id: true,
        name: true,
        slug: true,
        email: true
      }
    });

    if (!targetUser) {
      return res.status(404).json({ error: 'Target user not found' });
    }

    // Prepare uinfo.json content
    const uinfoData = {
      userId: targetUser.id,
      userName: targetUser.name,
      userSlug: targetUser.slug,
      assignedAt: new Date().toISOString(),
      nextJumpNumber: body.nextJumpNumber || 1
    };

    // Create the command in the queue
    const command = await prisma.deviceCommandQueue.create({
      data: {
        commandType: CommandType.ASSIGN,
        targetDeviceId: deviceId,
        sendingUserId: req.user!.id,
        commandData: uinfoData
      }
    });

    // Update device ownership in database
    await prisma.device.update({
      where: { id: deviceId },
      data: {
        ownerId: targetUser.id,
        state: 'ACTIVE'
      }
    });

    return res.status(200).json({
      message: `Device assignment queued for ${device.name}`,
      commandId: command.id,
      assignedTo: {
        id: targetUser.id,
        name: targetUser.name,
        email: targetUser.email
      }
    });

  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({ error: 'Invalid request data', details: error });
    }

    console.error('Assign device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});