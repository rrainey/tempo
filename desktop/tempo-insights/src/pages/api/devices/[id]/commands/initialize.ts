// /pages/api/devices/[id]/commands/initialize.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../../lib/auth/middleware';
import { PrismaClient, CommandType } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const bodySchema = z.object({
  pcbVersion: z.string().optional().default('1.0')
});

// Helper to get or create app settings
async function getNextDeviceId(): Promise<string> {
  // Check if we have an app settings entry for device IDs
  let settings = await prisma.appSettings.findFirst({
    where: { key: 'lastDeviceId' }
  });

  if (!settings) {
    // Create initial settings
    settings = await prisma.appSettings.create({
      data: {
        key: 'lastDeviceId',
        value: '0010' // Starting value as specified
      }
    });
  }

  // Parse current value and increment
  const currentId = parseInt(settings.value, 16);
  const nextId = currentId + 1;
  const nextIdHex = nextId.toString(16).toUpperCase().padStart(4, '0');

  // Update settings with new value
  await prisma.appSettings.update({
    where: { id: settings.id },
    data: { value: nextIdHex }
  });

  return nextIdHex;
}

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { id: deviceId } = req.query;
    
    if (!deviceId || typeof deviceId !== 'string') {
      return res.status(400).json({ error: 'Invalid device ID' });
    }

    // Only admins can initialize devices
    if (req.user!.role !== 'ADMIN' && req.user!.role !== 'SUPER_ADMIN') {
      return res.status(403).json({ error: 'Only administrators can initialize devices' });
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

    // Check if device is already initialized
    if (device.name !== 'Tempo-BT' && !device.name.startsWith('Tempo-BT-unprovisioned')) {
      return res.status(400).json({ 
        error: 'Device is already initialized',
        currentName: device.name
      });
    }

    // Check if device is online
    const cutoffTime = new Date(Date.now() - 5 * 60 * 1000); // 5 minutes
    const isOnline = device.lastSeen && device.lastSeen > cutoffTime;

    if (!isOnline) {
      return res.status(400).json({ error: 'Device is offline' });
    }

    // Get next device ID
    const deviceIdHex = await getNextDeviceId();
    const newDeviceName = `Tempo-BT-${deviceIdHex}`;

    // Create the command in the queue
    const command = await prisma.deviceCommandQueue.create({
      data: {
        commandType: CommandType.INITIALIZE,
        targetDeviceId: deviceId,
        sendingUserId: req.user!.id,
        commandData: {
          newName: newDeviceName,
          deviceId: deviceIdHex,
          pcbVersion: body.pcbVersion
        }
      }
    });

    return res.status(200).json({
      message: `Initialize command queued for device`,
      commandId: command.id,
      newName: newDeviceName,
      deviceId: deviceIdHex
    });

  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({ error: 'Invalid request data', details: error });
    }

    console.error('Initialize device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});