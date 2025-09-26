// Save as: src/pages/api/devices/[id]/commands/index.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const querySchema = z.object({
  limit: z.string().transform(Number).pipe(z.number().min(1).max(100)).optional().default(20),
  status: z.enum(['QUEUED', 'SENDING', 'COMPLETED', 'DEVICE_ERROR', 'DEVICE_TIMEOUT']).optional()
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method === 'GET') {
    // List commands for a device
    try {
      const { id: deviceId } = req.query;
      
      if (!deviceId || typeof deviceId !== 'string') {
        return res.status(400).json({ error: 'Invalid device ID' });
      }

      // Parse query parameters
      const query = querySchema.parse(req.query);

      // Get the device and check permissions
      const device = await prisma.device.findUnique({
        where: { id: deviceId }
      });

      if (!device) {
        return res.status(404).json({ error: 'Device not found' });
      }

      // Check if user has permission
      const isAdmin = req.user!.role === 'ADMIN' || req.user!.role === 'SUPER_ADMIN';
      const isOwner = device.ownerId === req.user!.id;
      const isBorrower = device.lentToId === req.user!.id;

      if (!isAdmin && !isOwner && !isBorrower) {
        return res.status(403).json({ error: 'You do not have permission to view commands for this device' });
      }

      // Build query conditions
      const where: any = {
        targetDeviceId: deviceId
      };

      if (query.status) {
        where.commandStatus = query.status;
      }

      // Get commands
      const commands = await prisma.deviceCommandQueue.findMany({
        where,
        include: {
          sendingUser: {
            select: {
              id: true,
              name: true,
              email: true
            }
          }
        },
        orderBy: {
          createdAt: 'desc'
        },
        take: query.limit
      });

      return res.status(200).json({
        commands: commands.map(cmd => ({
          id: cmd.id,
          commandType: cmd.commandType,
          commandStatus: cmd.commandStatus,
          commandData: cmd.commandData,
          responseData: cmd.responseData,
          errorMessage: cmd.errorMessage,
          createdAt: cmd.createdAt,
          startedAt: cmd.startedAt,
          completedAt: cmd.completedAt,
          sendingUser: cmd.sendingUser
        }))
      });

    } catch (error) {
      if (error instanceof z.ZodError) {
        return res.status(400).json({ error: 'Invalid query parameters', details: error });
      }

      console.error('List commands error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }

  } else if (req.method === 'DELETE') {
    // Delete a specific command
    const { id: deviceId, commandId } = req.query;
    
    if (!deviceId || typeof deviceId !== 'string' || !commandId || typeof commandId !== 'string') {
      return res.status(400).json({ error: 'Invalid device ID or command ID' });
    }

    try {
      // Get the command
      const command = await prisma.deviceCommandQueue.findUnique({
        where: { id: commandId },
        include: {
          targetDevice: true
        }
      });

      if (!command) {
        return res.status(404).json({ error: 'Command not found' });
      }

      if (command.targetDeviceId !== deviceId) {
        return res.status(400).json({ error: 'Command does not belong to this device' });
      }

      // Check if user has permission
      const isAdmin = req.user!.role === 'ADMIN' || req.user!.role === 'SUPER_ADMIN';
      const isSender = command.sendingUserId === req.user!.id;

      if (!isAdmin && !isSender) {
        return res.status(403).json({ error: 'You do not have permission to delete this command' });
      }

      // Only allow deletion of completed/failed commands
      if (!['COMPLETED', 'DEVICE_ERROR', 'DEVICE_TIMEOUT'].includes(command.commandStatus)) {
        return res.status(400).json({ error: 'Can only delete completed or failed commands' });
      }

      // Delete the command
      await prisma.deviceCommandQueue.delete({
        where: { id: commandId }
      });

      return res.status(200).json({ message: 'Command deleted successfully' });

    } catch (error) {
      console.error('Delete command error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }

  } else {
    return res.status(405).json({ error: 'Method not allowed' });
  }
});