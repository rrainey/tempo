import { requireAdmin, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient, DeviceState } from '@prisma/client';

const prisma = new PrismaClient();

export default requireAdmin(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { id } = req.query;

  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid device ID' });
  }

  try {
    // Get the device
    const device = await prisma.device.findUnique({
      where: { id },
      include: {
        _count: {
          select: {
            jumpLogs: true,
          },
        },
      },
    });

    if (!device) {
      return res.status(404).json({ error: 'Device not found' });
    }

    // TODO: In Phase 8, this will send actual Bluetooth command to reset device
    // For now, just update the database state
    console.log(`[UNPROVISION] Unprovisioning device ${device.name} (${device.bluetoothId})`);

    // Update device state to PROVISIONING and clear lentTo
    const updatedDevice = await prisma.device.update({
      where: { id },
      data: {
        state: DeviceState.PROVISIONING,
        lentToId: null,
        lastSeen: new Date(),
      },
    });

    return res.status(200).json({
      success: true,
      message: `Device ${device.name} has been unprovisioned`,
      device: {
        id: updatedDevice.id,
        name: updatedDevice.name,
        bluetoothId: updatedDevice.bluetoothId,
        state: updatedDevice.state,
        jumpLogsCount: device._count.jumpLogs,
      },
    });
  } catch (error) {
    console.error('Unprovision device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
